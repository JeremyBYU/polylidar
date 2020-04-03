#include "Polylidar/Polylidar.hpp"
#include "Polylidar/Core.hpp"

#include "Eigen/Dense"

using RowMatrixX3d = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RowMatrixX3lui = Eigen::Matrix<size_t, Eigen::Dynamic, 3, Eigen::RowMajor>;

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

std::tuple<Planes, Polygons> Polylidar3D::ExtractPlanesAndPolygonsOptimized(MeshHelper::HalfEdgeTriangulation& mesh,
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
                                                                             const Matrix<double>& plane_normals)
{
    auto plane_data_list = Utility::CreateMultiplePlaneDataFromNormals(plane_normals);
    // Create tri_set
    size_t max_triangles = mesh.triangles.rows;
    std::vector<uint8_t> tri_set(max_triangles, ZERO_UINT8);
    // Check the entire triset from the start.
    CreateTriSet3OptimizedForMultiplePlanes(tri_set, mesh, plane_data_list);
    // vectors for our planes and polygons, each element is for each normal to be expanded upon
    PlanesGroup planes_group;
    PolygonsGroup polygons_group;

    for (auto& plane_data : plane_data_list)
    {
        auto planes = ExtractPlanes(mesh, tri_set, plane_data, true);
        auto polygons = Core::ExtractConcaveHulls(planes, mesh, plane_data, min_hole_vertices);
        planes_group.emplace_back(std::move(planes));
        polygons_group.emplace_back(std::move(polygons));
    }
    return std::make_tuple(std::move(planes_group), std::move(polygons_group));
}

// Planes Polylidar3D::ExtractPlanesOptimized(MeshHelper::HalfEdgeTriangulation& mesh, std::vector<uint8_t>& tri_set,
//                                            PlaneData& plane_data)
// {
//     Planes planes;
//     size_t max_triangles = mesh.triangles.rows;

//     for (size_t t = 0; t < max_triangles; t++)
//     {
//         if (tri_set[t] == plane_data.normal_id)
//         {

//             planes.emplace_back();                       // construct empty vector inside planes
//             auto& plane_set = planes[planes.size() - 1]; // retrieve this newly created vector
//             Core::ExtractMeshSet(mesh, tri_set, t, plane_set, plane_data.normal_id);
//             if (!Core::PassPlaneConstraints(plane_set, min_triangles))
//             {
//                 planes.pop_back();
//             }
//         }
//     }
//     return planes;
// }

Planes Polylidar3D::ExtractPlanes(MeshHelper::HalfEdgeTriangulation& mesh, std::vector<uint8_t>& tri_set,
                                  PlaneData& plane_data, bool tri_set_finished)
{
    Planes planes;
    size_t max_triangles = mesh.triangles.rows;
    // If tri_set is not finished, then construct it
    if (!tri_set_finished)
    {
        if (mesh.vertices.cols == 2)
        {
            // std::cout << "Using TriSet2" << std::endl;
            CreateTriSet2(tri_set, mesh);
        }
        else
        {
            CreateTriSet3Optimized(tri_set, mesh, plane_data);
        }
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
    int numTriangles = static_cast<int>(mesh.triangles.rows);
// Ensure that each thread has at least PL_OMP_ELEM_PER_THREAD_TRISET
// Experimentation has found that too many threads will kill this loop if not enough work is present
#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
    num_threads = std::max(1, num_threads);
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (int t = 0; t < numTriangles; t++)
    {
        if (tri_set[t] != ZERO_UINT8) continue;
        tri_set[t] = Utility::ValidateTriangle2D(t, mesh, alpha, lmax) ? ONE_UINT8 : MAX_UINT8;
    }
}

inline RowMatrixX3d CreatePlaneNormalMatrix(std::vector<PlaneData>& plane_data_list)
{
    RowMatrixX3d a(plane_data_list.size(), 3);
    for (int i = 0; i < static_cast<int>(plane_data_list.size()); ++i)
    {
        auto& pn = plane_data_list[i].plane_normal;
        a.row(i) = Eigen::Vector3d(pn[0], pn[1], pn[2]);
    }
    return a;
}

void Polylidar3D::CreateTriSet3OptimizedForMultiplePlanes(std::vector<uint8_t>& tri_set,
                                                          MeshHelper::HalfEdgeTriangulation& mesh,
                                                          std::vector<PlaneData>& plane_data_list)
{
    int numTriangles = static_cast<int>(mesh.triangles.rows);
    RowMatrixX3d plane_normal_matrix = CreatePlaneNormalMatrix(plane_data_list);
    // Map to Eigen data types
    Eigen::Map<RowMatrixX3d> vertices_e(mesh.vertices.ptr, mesh.vertices.rows, 3);
    Eigen::Map<RowMatrixX3lui> triangles_e(mesh.triangles.ptr, mesh.triangles.rows, 3);
    Eigen::Map<RowMatrixX3d> triangles_normals_e(mesh.triangle_normals.ptr, mesh.triangle_normals.rows, 3);

    // std::cout << "Calling CreateTriSet3OptimizedForMultiplePlanes" << std::endl;

#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
    num_threads = std::max(1, num_threads);
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (int t = 0; t < numTriangles; t++)
    {
        int idx = 0;
        // SIGNIFICANT slow down compared to raw for loop
        // auto this_normal = triangles_normals_e.row(t).transpose();
        // auto maxDotProduct = (plane_normal_matrix * this_normal).maxCoeff(&idx);
        double maxDotProduct = -1.0;
        for(int i = 0; i < static_cast<int>(plane_data_list.size()); ++i)
        {
            auto dotproduct = triangles_normals_e.row(t).dot(plane_normal_matrix.row(i));
            if (dotproduct > maxDotProduct)
            {
                idx = i;
                maxDotProduct = dotproduct;
            }
        }

        uint8_t valid2D = Utility::ValidateTriangle2D(t, mesh, alpha, lmax) ? ZERO_UINT8 : MAX_UINT8;
        uint8_t valid3D = std::abs(maxDotProduct) > norm_thresh_min ? plane_data_list[idx].normal_id : ZERO_UINT8;
        tri_set[t] = valid2D | valid3D;
    }
}

void Polylidar3D::CreateTriSet3Optimized(std::vector<uint8_t>& tri_set, MeshHelper::HalfEdgeTriangulation& mesh,
                                         PlaneData& plane_data)
{
    // std::cout << "Calling CreateTriSet3Optimized" << std::endl;
    int numTriangles = static_cast<int>(mesh.triangles.rows);
    Eigen::Vector3d plane_normal(plane_data.plane_normal[0], plane_data.plane_normal[1], plane_data.plane_normal[2]);
    // Map to Eigen data types
    Eigen::Map<RowMatrixX3d> vertices_e(mesh.vertices.ptr, mesh.vertices.rows, 3);
    Eigen::Map<RowMatrixX3lui> triangles_e(mesh.triangles.ptr, mesh.triangles.rows, 3);
    Eigen::Map<RowMatrixX3d> triangles_normals_e(mesh.triangle_normals.ptr, mesh.triangle_normals.rows, 3);

#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
    num_threads = std::max(1, num_threads);
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (int t = 0; t < numTriangles; t++)
    {
        if (tri_set[t] != ZERO_UINT8) continue;
        uint8_t valid2D = Utility::ValidateTriangle2D(t, mesh, alpha, lmax) ? ZERO_UINT8 : MAX_UINT8;
        uint8_t valid3D = std::abs(triangles_normals_e.row(t).dot(plane_normal)) > norm_thresh_min
                              ? plane_data.normal_id
                              : ZERO_UINT8;
        tri_set[t] = valid2D | valid3D;
    }
}

void Polylidar3D::CreateTriSet3(std::vector<uint8_t>& tri_set, MeshHelper::HalfEdgeTriangulation& mesh,
                                PlaneData& plane_data)
{
    // std::cout << "Calling CreateTriSet3" << std::endl;
    int numTriangles = static_cast<int>(mesh.triangles.rows);

// Ensure that each thread has at least PL_OMP_ELEM_PER_THREAD_TRISET
// Experimentation has found that too many threads will kill this loop if not enough work is present
#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
    num_threads = std::max(1, num_threads);
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (int t = 0; t < numTriangles; t++)
    {
        if (tri_set[t] != ZERO_UINT8) continue;
        uint8_t valid2D = Utility::ValidateTriangle2D(t, mesh, alpha, lmax) ? ZERO_UINT8 : MAX_UINT8;
        uint8_t valid3D = Utility::ValidateTriangle3D(static_cast<size_t>(t), mesh, z_thresh, norm_thresh,
                                                      norm_thresh_min, plane_data.plane_normal)
                              ? plane_data.normal_id
                              : ZERO_UINT8;
        tri_set[t] = valid2D | valid3D;
    }
}

} // namespace Polylidar