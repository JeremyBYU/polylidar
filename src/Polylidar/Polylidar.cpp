#include "Polylidar/Polylidar.hpp"
#include "Polylidar/Core.hpp"

#include "marl/defer.h"
#include "marl/scheduler.h"
#include "marl/waitgroup.h"

#include "Eigen/Dense"
// These eigen types are used ONLY used in  CreateTriSet3 Functions
// Types are kept here in case I decide to remove eigen later
using RowMatrixX3d = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using RowMatrixX3lui = Eigen::Matrix<size_t, Eigen::Dynamic, 3, Eigen::RowMajor>;

namespace Polylidar {

Polylidar3D::Polylidar3D(const double _alpha, const double _lmax, const size_t _min_triangles,
                         const size_t _min_hole_vertices, const double _z_thresh, const double _norm_thresh,
                         const double _norm_thresh_min, const int _task_threads)
    : alpha(_alpha),
      lmax(_lmax),
      min_triangles(_min_triangles),
      min_hole_vertices(_min_hole_vertices),
      z_thresh(_z_thresh),
      norm_thresh(_norm_thresh),
      norm_thresh_min(_norm_thresh_min),
      task_threads(_task_threads)
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

std::tuple<Planes, Polygons> Polylidar3D::ExtractPlanesAndPolygonsOptimized(MeshHelper::HalfEdgeTriangulation& mesh,
                                                                            const std::array<double, 3> plane_normal)
{
    auto plane_normal_ptr = const_cast<double*>(plane_normal.data()); // I wont change it or mutate it
    const Matrix<double> plane_normals(plane_normal_ptr, 1, 3);

    // Only one group will be returned because we have one normal
    PlanesGroup planes_group;
    PolygonsGroup polygons_group;

    std::tie(planes_group, polygons_group) = ExtractPlanesAndPolygonsOptimized(mesh, plane_normals);

    Planes planes = std::move(planes_group[0]);
    Polygons polygons = std::move(polygons_group[0]);

    return std::make_tuple(std::move(planes), std::move(polygons));
}

std::tuple<PlanesGroup, PolygonsGroup>
Polylidar3D::ExtractPlanesAndPolygonsOptimized(MeshHelper::HalfEdgeTriangulation& mesh,
                                               const Matrix<double>& plane_normals)
{
    auto plane_data_list = Utility::CreateMultiplePlaneDataFromNormals(plane_normals);

    size_t max_triangles = mesh.triangles.rows;
    std::vector<uint8_t> tri_set(max_triangles, ZERO_UINT8);
    // Check the entire triset from the start.
    // Utility::Timer timer(true);
    CreateTriSet3OptimizedForMultiplePlanes(tri_set, mesh, plane_data_list);
    // std::cout << "Create TriSet3Optimized took: " << timer << " us" << std::endl;

    int number_of_groups = static_cast<int>(plane_data_list.size());
    // reserve size to guarantee pointer/reference stability
    PlanesGroup planes_group(number_of_groups);
    PolygonsGroup polygons_group(number_of_groups);

    // use marl for dynamic task creation and exectution
    marl::Scheduler scheduler;
    scheduler.bind();
    scheduler.setWorkerThreadCount(task_threads);
    defer(scheduler.unbind()); // Automatically unbind before returning.

    // We will extract each plane group using dynamic tasks
    marl::WaitGroup plane_data_wg(static_cast<int>(plane_data_list.size()));

    for (int i = 0; i < number_of_groups; ++i)
    {
        marl::schedule([this, i, &mesh, &tri_set, &plane_data_list, &planes_group, &polygons_group, plane_data_wg] {
            defer(plane_data_wg.done());           // Decrement plane_data_wg when task is done
            auto& plane_data = plane_data_list[i]; // get plane data

            Planes planes;     // will contain the planes extracted on this normal
            Polygons polygons; // will contain the associated polygons for these planes

            std::tie(planes, polygons) = ExtractPlanesWithTasks(mesh, tri_set, plane_data);

            planes_group[i] = std::move(planes);
            polygons_group[i] = std::move(polygons);
        });
    }
    // wait here until all plane groups have been processed
    plane_data_wg.wait();

    return std::make_tuple(std::move(planes_group), std::move(polygons_group));
}

std::tuple<PlanesGroup, PolygonsGroup> Polylidar3D::ExtractPlanesAndPolygons(MeshHelper::HalfEdgeTriangulation& mesh,
                                                                             const Matrix<double>& plane_normals)
{
    auto plane_data_list = Utility::CreateMultiplePlaneDataFromNormals(plane_normals);
    // Create tri_set
    size_t max_triangles = mesh.triangles.rows;
    std::vector<uint8_t> tri_set(max_triangles, ZERO_UINT8);
    // Check the entire triset from the start.
    // Utility::Timer timer(true);
    CreateTriSet3OptimizedForMultiplePlanes(tri_set, mesh, plane_data_list);
    // std::cout << "Create TriSet3Optimized took: " << timer << " us" << std::endl;
    // vectors for our planes and polygons, each element is for each normal to be expanded upon
    PlanesGroup planes_group;
    PolygonsGroup polygons_group;

    // All done in serial
    for (auto& plane_data : plane_data_list)
    {
        auto planes = ExtractPlanes(mesh, tri_set, plane_data, true);
        auto polygons = Core::ExtractConcaveHulls(planes, mesh, plane_data, min_hole_vertices);
        planes_group.emplace_back(std::move(planes));
        polygons_group.emplace_back(std::move(polygons));
    }

    return std::make_tuple(std::move(planes_group), std::move(polygons_group));
}

std::tuple<Planes, Polygons> Polylidar3D::ExtractPlanesWithTasks(MeshHelper::HalfEdgeTriangulation& mesh,
                                                                 std::vector<uint8_t>& tri_set, PlaneData& plane_data)
{

    // Planes planes; // will contain the planes extracted on this normal
    Polygons polygons; // will contain the associated polygons for these planes

    // need this to guarantee pointer/reference stability
    // the other option is to keep using vector and reserve a large amount of memory up front (e.g. 1000)
    // this works, and is not *too* expensive (because the element of planes are vectors themselves)
    // however, that will break if the amount of planes extracted exceeds 1000, and a reallocation occurs and
    // reference passed in other threads are no longer valid
    std::deque<VUI> planes_deque;

    // We will extract polygons using created dynamic tasks
    // This is a thread_safe counter to see the amount of polygons tasks created and being worked on
    marl::WaitGroup polygons_wg(0);

    // planes.reserve(1000); // this wil the first strategy, works and is cheap but is more of a hack
    // polygons.reserve(1000); // may revert back if the deque strategy does not solve the problem

    size_t max_triangles = mesh.triangles.rows;
    int plane_counter = 0;
    for (size_t t = 0; t < max_triangles; t++)
    {
        if (tri_set[t] == plane_data.normal_id)
        {

            VUI plane_set;
            // Plane extraction occurs in serial, but polygon extraction is in parallel through tasks
            Core::ExtractMeshSet(mesh, tri_set, t, plane_set, plane_data.normal_id);
            if (Core::PassPlaneConstraints(plane_set, min_triangles))
            {
                planes_deque.emplace_back(std::move(plane_set));
                plane_counter++;

                polygons.emplace_back();
                polygons_wg.add(1);
                marl::schedule([this, &planes_deque, plane_counter, &mesh, &plane_data, &polygons, polygons_wg] {
                    defer(polygons_wg.done()); // Decrement polygons_wg when task is done
                    auto& plane =
                        planes_deque[plane_counter - 1]; // get the plane, reference is stable when using deque
                    auto polygon = Core::ExtractConcaveHull(plane, mesh, plane_data, min_hole_vertices);
                    polygons[plane_counter - 1] = std::move(polygon);
                });
            }
        }
    }

    // There could be polygon not finished being processed in the fibers
    // wait here until all polygons have been processed
    polygons_wg.wait();
    // move the planes from the deque to the vector, should be fast/no copy
    Planes planes{std::make_move_iterator(std::begin(planes_deque)), std::make_move_iterator(std::end(planes_deque))};

    return std::make_tuple(std::move(planes), std::move(polygons));
}

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


#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
    num_threads = std::max(1, num_threads);
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (int t = 0; t < numTriangles; t++)
    {
        int idx = 0;
        double maxDotProduct = -1.0;
        // SIGNIFICANT slow down compared to raw for loop
        // auto this_normal = triangles_normals_e.row(t).transpose();
        // auto maxDotProduct = (plane_normal_matrix * this_normal).maxCoeff(&idx);
        for (int i = 0; i < static_cast<int>(plane_data_list.size()); ++i)
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