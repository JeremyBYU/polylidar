// These benchmarks explicitely time plane and polygon extraction
// It also measures how CPU scores scale with the problem
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdlib>

#include <Polylidar/Polylidar.hpp>
#include <benchmark/benchmark.h>
#include <Open3D/IO/ClassIO/ImageIO.h>
#include <Open3D/IO/ClassIO/TriangleMeshIO.h>
#include <Open3D/Geometry/Image.h>
#include <Open3D/Geometry/TriangleMesh.h>
#include <omp.h>

const std::string SPARSE_MESH = "./fixtures/meshes/basement_chair_5cm.ply";
const std::string DENSE_MESH = "./fixtures/meshes/dense_first_floor_map_smoothed.ply";

using namespace Polylidar;
namespace o3d = open3d;
using TriMesh = MeshHelper::HalfEdgeTriangulation;

static constexpr int max_threads = 8;

inline void CustomArguments_Paper(benchmark::internal::Benchmark* b)
{
    for (int i = 1; i <= 4; i += 1)
        for (int j = 1; j <= max_threads; j += 1) b->Args({i, j});
}

class SparseMeshPaper : public benchmark::Fixture
{
  public:
    std::shared_ptr<o3d::geometry::TriangleMesh> sparse_mesh_o3d;
    TriMesh sparse_mesh;
    // Eigen::Matrix3d mesh_rotation;
    void SetUp(const ::benchmark::State& state)
    {

        // Sparse Mesh In O3D Format
        sparse_mesh_o3d = o3d::io::CreateMeshFromFile(SPARSE_MESH);
        // Sparse Mesh In HalfEdgeTriangulation
        auto triangles_ptr = reinterpret_cast<int*>(sparse_mesh_o3d->triangles_.data());
        auto num_triangles = sparse_mesh_o3d->triangles_.size();
        auto vertices_ptr = reinterpret_cast<double*>(sparse_mesh_o3d->vertices_.data());
        auto num_vertices = sparse_mesh_o3d->vertices_.size();
        Matrix<int> triangles(triangles_ptr, num_triangles, 3);
        Matrix<double> vertices(vertices_ptr, num_vertices, 3);

        sparse_mesh = MeshHelper::CreateTriMeshCopy(vertices, triangles, true);
    }
};

class DenseMeshPaper : public benchmark::Fixture
{
  public:
    std::shared_ptr<o3d::geometry::TriangleMesh> dense_mesh_o3d;
    TriMesh dense_mesh;
    // Eigen::Matrix3d mesh_rotation;
    void SetUp(const ::benchmark::State& state)
    {

        // Sparse Mesh In O3D Format
        dense_mesh_o3d = o3d::io::CreateMeshFromFile(DENSE_MESH);
        auto triangles_ptr = reinterpret_cast<int*>(dense_mesh_o3d->triangles_.data());
        auto num_triangles = dense_mesh_o3d->triangles_.size();
        auto vertices_ptr = reinterpret_cast<double*>(dense_mesh_o3d->vertices_.data());
        auto num_vertices = dense_mesh_o3d->vertices_.size();
        Matrix<int> triangles(triangles_ptr, num_triangles, 3);
        Matrix<double> vertices(vertices_ptr, num_vertices, 3);

        dense_mesh = MeshHelper::CreateTriMeshCopy(vertices, triangles, true);
    }
};

BENCHMARK_DEFINE_F(SparseMeshPaper, BM_Paper)
(benchmark::State& st)
{
    std::vector<std::array<double, 3>> normals = {{-0.0296, 0.0003, 0.9996},
                                                  {-0.9534, 0.3016, 0.0077},
                                                  {0.9365, -0.3371, 0.0963},
                                                  {0.3054, 0.9522, -0.012},
                                                  {0.0111 - 0.0113 - 0.9999}};

    int num_normals = st.range(0);
    int num_threads = st.range(1);
    omp_set_num_threads(num_threads);
    Polylidar3D pl3d(0.0, 0.1, 1000, 6, 0.03, 0.95, 0.92, num_threads);
    const Matrix<double> normals_mat((double*)(normals.data()), num_normals, 3);
    for (auto _ : st)
    {
        auto planes_and_polygons = pl3d.ExtractPlanesAndPolygonsOptimized(sparse_mesh, normals_mat);
        benchmark::DoNotOptimize(planes_and_polygons);
    }
}

BENCHMARK_DEFINE_F(DenseMeshPaper, BM_Paper)
(benchmark::State& st)
{
    std::vector<std::array<double, 3>> normals = {
        {0.0123, 0.0021, 0.9999}, {0.2241, -0.9744, 0.0171}, {-0.2207, 0.9752, -0.0144}, {0.9612, 0.2756, -0.0129}};

    int num_normals = st.range(0);
    int num_threads = st.range(1);
    omp_set_num_threads(num_threads);
    Polylidar3D pl3d(0.0, 0.1, 1000, 6, 0.03, 0.95, 0.92, num_threads);
    const Matrix<double> normals_mat((double*)(normals.data()), num_normals, 3);
    for (auto _ : st)
    {
        auto planes_and_polygons = pl3d.ExtractPlanesAndPolygonsOptimized(dense_mesh, normals_mat);
        benchmark::DoNotOptimize(planes_and_polygons);
    }
}

BENCHMARK_REGISTER_F(SparseMeshPaper, BM_Paper)
    ->Apply(CustomArguments_Paper)
    ->UseRealTime()
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(DenseMeshPaper, BM_Paper)
    ->Apply(CustomArguments_Paper)
    ->UseRealTime()
    ->Unit(benchmark::kMillisecond);
