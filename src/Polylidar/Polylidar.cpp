#include "Polylidar/Polylidar.hpp"
#include "Polylidar/Core.hpp"

namespace Polylidar {

Polylidar3D::Polylidar3D(const double _alpha, const double _lmax, const size_t _min_triangles,
                         const size_t _min_hole_vertices, const double _z_thresh, const double _norm_thresh,
                         const double _norm_thresh_min)
    : alpha(_alpha),
      lmax(_lmax),
      min_triangles(_min_triangles),
      min_hole_vertices(_min_hole_vertices),
      z_thresh(_z_thresh),
      norm_thresh(_norm_thresh),
      norm_thresh_min(_norm_thresh_min)
{
}

std::tuple<MeshHelper::HalfEdgeTriangulation, Planes, Polygons>
Polylidar3D::ExtractPlanesAndPolygons(const Matrix<double>& points, const std::array<double, 3> plane_normal)
{
    // Triangulate points in only 2 dimensions (x,y)
    Utility::Timer timer(true);
    Delaunator::Delaunator mesh(points);
    mesh.triangulate();

    // 2.5D Delaunay Triangulation
    if (mesh.vertices.cols > 2)
    {
        mesh.ComputeTriangleNormals();
    }

    // Create Plane Data Structure, informs Polylidar which normal to extract on
    PlaneData plane_data{plane_normal};
    Utility::UpdatePlaneDataWithRotationInformation(plane_data);

    // Extract planes from the mesh from desired plane normal
    timer.Reset();
    size_t max_triangles = mesh.triangles.rows;
    std::vector<uint8_t> tri_set(max_triangles, ZERO_UINT8);
    auto planes = ExtractPlanes(mesh, tri_set, plane_data);

    Polygons polygons;
    timer.Reset();
    polygons = Core::ExtractConcaveHulls(planes, mesh, plane_data, min_hole_vertices);
    return std::make_tuple(std::move(mesh), std::move(planes), std::move(polygons));
}

std::tuple<Planes, Polygons> Polylidar3D::ExtractPlanesAndPolygons(MeshHelper::HalfEdgeTriangulation& mesh,
                                                                   const std::array<double, 3> plane_normal)
{
    Utility::Timer timer(true);
    // Create Plane Data Structure, informs Polylidar which normal to extract on
    PlaneData plane_data{plane_normal};
    Utility::UpdatePlaneDataWithRotationInformation(plane_data);

    // Extract planes from the mesh from desired plane normal
    timer.Reset();
    size_t max_triangles = mesh.triangles.rows;
    std::vector<uint8_t> tri_set(max_triangles, ZERO_UINT8);
    auto planes = ExtractPlanes(mesh, tri_set, plane_data);

    Polygons polygons;
    timer.Reset();
    polygons = Core::ExtractConcaveHulls(planes, mesh, plane_data, min_hole_vertices);
    return std::make_tuple(std::move(planes), std::move(polygons));
}

std::tuple<PlanesGroup, PolygonsGroup> Polylidar3D::ExtractPlanesAndPolygons(MeshHelper::HalfEdgeTriangulation& mesh,
                                                                const Matrix<double> &plane_normals)
{
    auto plane_data_list = Utility::CreateMultiplePlaneDataFromNormals(plane_normals);
    // Create tri_set
    size_t max_triangles = mesh.triangles.rows;
    std::vector<uint8_t> tri_set(max_triangles, ZERO_UINT8);
    // vectors for our planes and polygons, each element is for each normal to be expanded upon
    PlanesGroup planes_group;
    PolygonsGroup polygons_group;

    for (auto& plane_data : plane_data_list)
    {
        auto planes = ExtractPlanes(mesh, tri_set, plane_data);
        auto polygons = Core::ExtractConcaveHulls(planes, mesh, plane_data, min_hole_vertices);
        planes_group.emplace_back(std::move(planes));
        polygons_group.emplace_back(std::move(polygons));
    }
    return std::make_tuple(std::move(planes_group), std::move(polygons_group));
}

Planes Polylidar3D::ExtractPlanes(MeshHelper::HalfEdgeTriangulation& mesh, std::vector<uint8_t>& tri_set,
                                  PlaneData& plane_data)
{
    Planes planes;
    size_t max_triangles = mesh.triangles.rows;

    if (mesh.vertices.cols == 2)
    {
        // std::cout << "Using TriSet2" << std::endl;
        CreateTriSet2(tri_set, mesh);
    }
    else
    {
        CreateTriSet3(tri_set, mesh, plane_data);
    }

    for (size_t t = 0; t < max_triangles; t++)
    {
        if (tri_set[t] == plane_data.normal_id)
        {

            planes.emplace_back();                       // construct empty vector inside planes
            auto& plane_set = planes[planes.size() - 1]; // retrieve this newly created vector
            Core::ExtractMeshSet(mesh, tri_set, t, plane_set, plane_data.normal_id);
            if (!Core::PassPlaneConstraints(plane_set, min_triangles))
            {
                planes.pop_back();
            }
        }
    }
    return planes;
}

void Polylidar3D::CreateTriSet2(std::vector<uint8_t>& tri_set, MeshHelper::HalfEdgeTriangulation& mesh)
{
    size_t& numTriangles = mesh.triangles.rows;
// Ensure that each thread has at least PL_OMP_ELEM_PER_THREAD_TRISET
// Experimentation has found that too many threads will kill this loop if not enough work is present
#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
    num_threads = std::max(1, num_threads);
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (size_t t = 0; t < numTriangles; t++)
    {
        if (tri_set[t] != ZERO_UINT8) continue;
        tri_set[t] = Utility::ValidateTriangle2D(t, mesh, alpha, lmax) ? ONE_UINT8 : MAX_UINT8;
    }
}

void Polylidar3D::CreateTriSet3(std::vector<uint8_t>& tri_set, MeshHelper::HalfEdgeTriangulation& mesh,
                                PlaneData& plane_data)
{
    size_t& numTriangles = mesh.triangles.rows;

// Ensure that each thread has at least PL_OMP_ELEM_PER_THREAD_TRISET
// Experimentation has found that too many threads will kill this loop if not enough work is present
#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
    num_threads = std::max(1, num_threads);
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (size_t t = 0; t < numTriangles; t++)
    {
        if (tri_set[t] != ZERO_UINT8) continue;
        uint8_t valid2D = Utility::ValidateTriangle2D(t, mesh, alpha, lmax) ? ZERO_UINT8 : MAX_UINT8;
        uint8_t valid3D =
            Utility::ValidateTriangle3D(t, mesh, z_thresh, norm_thresh, norm_thresh_min, plane_data.plane_normal)
                ? plane_data.normal_id
                : ZERO_UINT8;
        tri_set[t] = valid2D | valid3D;
    }
}

} // namespace Polylidar