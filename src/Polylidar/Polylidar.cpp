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

Polygon Polylidar3D::ExtractConcaveHull(VUI plane, MeshHelper::HalfEdgeTriangulation& mesh, PlaneData& plane_data,
                                        size_t min_hole_vertices_)
{
    Polygon poly;
    // point hash map
    unordered_map<size_t, std::vector<size_t>> pointHash;
    // hash of all empty border half edges
    unordered_map<size_t, size_t> edgeHash;
    // the left and right most extreme points
    ExtremePoint xPoint;

    // 99.6% of the time inside extractConcaveHull is inside constructPointHash
    // auto before = std::chrono::high_resolution_clock::now();
    Core::ConstructPointHash(plane, mesh, pointHash, edgeHash, xPoint, plane_data);
    // auto after = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "ConstructPointHash - Total (ms): " << elapsed.count() << std::endl;

    // std::vector<size_t> bucket_sizes;
    // for(auto i = 0; i < edgeHash.bucket_count(); ++i)
    // {
    //   auto bucket_size = edgeHash.bucket_size(i);
    //   bucket_sizes.push_back(bucket_size);
    // }
    // std::cout << bucket_sizes << std::endl;

    auto startingHalfEdge = xPoint.xr_he;
    // Error checking, just in case the extreme right point has a hole connected to it
    auto& nextEdges = pointHash[xPoint.xr_pi];
    if (nextEdges.size() > 1)
    {
        // std::cout << "Starting half edge has a hole on it " << std::endl;
        // std::cout << "Right extreme point is connected to a hole. Determining correct edge..." << std::endl;
        // std::cout << "xPoint: " << xPoint << std::endl;
        // std::cout << "Plane size: " << plane.size() << std::endl;
        // std::cout << "Point Hash size: " << pointHash.size() << std::endl;
        // std::cout << "Edge Hash size: " << edgeHash.size() << std::endl;
        startingHalfEdge =
            Core::GetHullEdge(UP_VECTOR, nextEdges, mesh, plane_data.rotation_matrix, plane_data.need_rotation, false);
    }
    auto startingPointIndex = xPoint.xr_pi;
    // std::cout << "Staring point index " << startingPointIndex << std::endl;;
    auto stopPoint = startingPointIndex;
    auto shell = Core::ConcaveSection(pointHash, edgeHash, mesh, startingHalfEdge, stopPoint, plane_data, false);
    auto holes = Core::ExtractInteriorHoles(pointHash, edgeHash, mesh, plane_data);

    holes.erase(
        std::remove_if(holes.begin(), holes.end(),
                       [&min_hole_vertices_](std::vector<size_t>& hole) { return hole.size() < min_hole_vertices_; }),
        holes.end());

    // std::vector<std::vector<size_t>> holes;
    poly.shell = std::move(shell);
    poly.holes = std::move(holes);
    return poly;
}

Polygons Polylidar3D::ExtractConcaveHulls(Planes planes, MeshHelper::HalfEdgeTriangulation& mesh, PlaneData& plane_data,
                                          size_t min_hole_vertices_)
{

    std::vector<Polygon> polygons;
    for (auto&& plane : planes)
    {
        Polygon poly = ExtractConcaveHull(plane, mesh, plane_data, min_hole_vertices_);
        polygons.push_back(std::move(poly));
    }
    return polygons;
}

std::tuple<MeshHelper::HalfEdgeTriangulation, Planes, Polygons>
Polylidar3D::ExtractPlanesAndPolygons(const Matrix<double>& points, const std::array<double, 3> plane_normal)
{
    // Triangulate points in only 2 dimensions (x,y)
    Utility::Timer timer(true);
    Delaunator::Delaunator mesh(points);
    std::cout << "Delaunay init took: " << timer << std::endl;
    mesh.triangulate();
    std::cout << "Delaunay took: " << timer << std::endl;

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
    std::cout << "Plane Extraction took: " << timer << std::endl;

    Polygons polygons;
    timer.Reset();
    polygons = ExtractConcaveHulls(planes, mesh, plane_data, min_hole_vertices);
    std::cout << "Concave Extraction took: " << timer << std::endl;
    // // after = std::chrono::high_resolution_clock::now();
    // // elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // // std::cout << "Polygon Hull Extraction took " << elapsed.count() << " milliseconds" << std::endl;
    return std::make_tuple(std::move(mesh), std::move(planes), std::move(polygons));
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
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (size_t t = 0; t < numTriangles; t++)
    {
        if (tri_set[t] != ZERO_UINT8) continue;
        tri_set[t] = Utility::ValidateTriangle2D(t, mesh) ? ONE_UINT8 : MAX_UINT8;
        // std::cout << unsigned(tri_set[t]) << std::endl;
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
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (size_t t = 0; t < numTriangles; t++)
    {
        if (tri_set[t] != ZERO_UINT8) continue;
        uint8_t valid2D = Utility::ValidateTriangle2D(t, mesh) ? ZERO_UINT8 : MAX_UINT8;
        // uint8_t valid3D = Utility::ValidateTriangle3D_Opt(t, delaunay, points, config) ? config.normalID :
        // ZERO_UINT8;
        uint8_t valid3D = valid2D;
        tri_set[t] = valid2D | valid3D;
    }
}

} // namespace Polylidar