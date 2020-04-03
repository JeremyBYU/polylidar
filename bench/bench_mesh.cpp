#include <iostream>
#include <sstream>
#include <vector>

#include <Polylidar/Polylidar.hpp>
#include <benchmark/benchmark.h>
#include <Open3D/IO/ClassIO/ImageIO.h>
#include <Open3D/IO/ClassIO/TriangleMeshIO.h>
#include <Open3D/Geometry/Image.h>
#include <Open3D/Geometry/TriangleMesh.h>
#include <omp.h>

const std::string DEFAULT_DEPTH_IMAGE = "./fixtures/realsense/depth/00000003_1580908154939.png";
const std::string SPARSE_MESH = "./fixtures/meshes/sparse_basement.ply";
const std::string DENSE_MESH = "./fixtures/meshes/dense_first_floor_map_smoothed.ply";

using namespace Polylidar;
namespace o3d = open3d;

using TriMesh = MeshHelper::HalfEdgeTriangulation;

class ImagesAndSparseMesh : public benchmark::Fixture
{
  public:
    std::shared_ptr<o3d::geometry::Image> image_z16;
    std::shared_ptr<o3d::geometry::Image> image_float;
    std::vector<double> intr_{307, 0, 0, 0, 307, 0, 204, 121, 1};
    Matrix<double> intr;
    std::vector<double> extr_{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    Matrix<double> extr;
    // Mesh Files from .ply files
    std::shared_ptr<o3d::geometry::TriangleMesh> sparse_mesh_o3d;
    TriMesh sparse_mesh;
    Eigen::Matrix3d mesh_rotation;
    void SetUp(const ::benchmark::State& state)
    {
        o3d::geometry::Image image_z16_;
        auto success = o3d::io::ReadImage(DEFAULT_DEPTH_IMAGE, image_z16_);
        image_z16 = std::make_shared<o3d::geometry::Image>(image_z16_);
        image_float = image_z16->ConvertDepthToFloatImage();
        intr.ptr = intr_.data(), intr.cols = 3;
        intr.rows = 3;
        extr = Matrix<double>(intr_.data(), 4, 4);

        // Rotation for mesh, camera axis to global frame
        mesh_rotation << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
        // Sparse Mesh In O3D Format
        sparse_mesh_o3d = o3d::io::CreateMeshFromFile(SPARSE_MESH);
        sparse_mesh_o3d->Rotate(mesh_rotation);
        // sparse_mesh_o3d->Rotate();
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

class DenseMesh : public benchmark::Fixture
{
  public:
    std::shared_ptr<o3d::geometry::TriangleMesh> dense_mesh_o3d;
    TriMesh dense_mesh;
    Eigen::Matrix3d mesh_rotation;
    void SetUp(const ::benchmark::State& state)
    {

        // Rotation for mesh, camera axis to global frame
        // Sparse Mesh In O3D Format
        dense_mesh_o3d = o3d::io::CreateMeshFromFile(DENSE_MESH);
        // dont need to rotate the smooth mesh
        // mesh_rotation << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
        // dense_mesh_o3d->Rotate(mesh_rotation);

        auto triangles_ptr = reinterpret_cast<int*>(dense_mesh_o3d->triangles_.data());
        auto num_triangles = dense_mesh_o3d->triangles_.size();
        auto vertices_ptr = reinterpret_cast<double*>(dense_mesh_o3d->vertices_.data());
        auto num_vertices = dense_mesh_o3d->vertices_.size();
        Matrix<int> triangles(triangles_ptr, num_triangles, 3);
        Matrix<double> vertices(vertices_ptr, num_vertices, 3);

        dense_mesh = MeshHelper::CreateTriMeshCopy(vertices, triangles, true);
    }
};

BENCHMARK_DEFINE_F(ImagesAndSparseMesh, BM_Create_PointCloud)
(benchmark::State& st)
{
    auto im_ptr = image_float->PointerAt<float>(0, 0);
    image_float->height_;
    Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
    for (auto _ : st)
    {
        auto points = MeshHelper::ExtractPointCloudFromFloatDepth(im, intr, extr, 2);
    }
}

BENCHMARK_DEFINE_F(ImagesAndSparseMesh, BM_Create_TriMesh)
(benchmark::State& st)
{
    auto im_ptr = image_float->PointerAt<float>(0, 0);
    image_float->height_;
    Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
    for (auto _ : st)
    {
        // ExtractPointCloudFromFloatDepth(im, intr, 1);
        auto triMesh = MeshHelper::ExtractTriMeshFromFloatDepth(im, intr, extr, 2, false);
    }
}

BENCHMARK_DEFINE_F(ImagesAndSparseMesh, BM_ComputeTriangleNormals)
(benchmark::State& st)
{
    auto im_ptr = image_float->PointerAt<float>(0, 0);
    image_float->height_;
    Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
    auto triMesh = MeshHelper::ExtractTriMeshFromFloatDepth(im, intr, extr, 2, false);

    for (auto _ : st)
    {
        std::vector<double> triangle_normals;
        MeshHelper::ComputeTriangleNormalsFromMatrix(triMesh.vertices, triMesh.triangles, triMesh.triangle_normals);
    }
}

BENCHMARK_DEFINE_F(ImagesAndSparseMesh, BM_CopyTriMesh)
(benchmark::State& st)
{
    auto triangles_ptr = reinterpret_cast<int*>(sparse_mesh_o3d->triangles_.data());
    auto num_triangles = sparse_mesh_o3d->triangles_.size();
    auto vertices_ptr = reinterpret_cast<double*>(sparse_mesh_o3d->vertices_.data());
    auto num_vertices = sparse_mesh_o3d->vertices_.size();

    Matrix<int> triangles(triangles_ptr, num_triangles, 3);
    Matrix<double> vertices(vertices_ptr, num_vertices, 3);
    // ALmost all the time is just extracting the half edges
    for (auto _ : st)
    {
        auto test = MeshHelper::CreateTriMeshCopy(vertices, triangles, false);
    }
}

BENCHMARK_DEFINE_F(ImagesAndSparseMesh, BM_ExtractPlanesAndPolygons)
(benchmark::State& st)
{
    Polylidar3D pl3d(0.0, 0.1, 1000, 6, 0.03, 0.95, 0.90);
    for (auto _ : st)
    {
        // ExtractPointCloudFromFloatDepth(im, intr, 1);
        auto planes_and_polygons = pl3d.ExtractPlanesAndPolygons(sparse_mesh, {{0.0, 0.0, 1.0}});
    }
}

// BENCHMARK_DEFINE_F(Images, BM_ExtractPlanesAndPolygons)
// (benchmark::State& st)
// {
//     // dict(alpha=0.0, lmax=0.10, minTriangles=1000,
//     //                         zThresh=0.03, normThresh=0.95, normThreshMin=0.90, minHoleVertices=6)
//     polylidar::Config config{3, 0.0, 0.0, 0.10, 1000, 6, 0.0, 0.03, 0.95, 0.90, 1.0, {{0, 0, 1}}};
//     for (auto _ : st)
//     {
//         std::vector<std::vector<size_t>> planes;
//         std::vector<polylidar::Polygon> polygons;
//         // ExtractPointCloudFromFloatDepth(im, intr, 1);
//         std::tie(planes, polygons) = polylidar::ExtractPlanesAndPolygonsFromMesh(sparse_mesh, config);
//     }
// }

BENCHMARK_DEFINE_F(ImagesAndSparseMesh, BM_ExtractPlanesAndPolygonsFromMultipleNormals)
(benchmark::State& st)
{
    std::vector<std::array<double, 3>> normals = {{-0.0192, -0.0199, -0.9996},
                                                  {0.0023, 0.0227, 0.9997},
                                                  {0.9995, -0.0213, -0.0227},
                                                  // { 0.9971,  0.0709, -0.0291},
                                                  {-0.0163, -0.9995, -0.0267},
                                                  // {0.0168, -0.2067,  0.9783},
                                                  {-0., 0.5988, 0.8009}};
    Polylidar3D pl3d(0.0, 0.1, 1000, 6, 0.03, 0.95, 0.90);
    const Matrix<double> normals_mat((double*)(normals.data()), st.range(0), 3);
    for (auto _ : st)
    {
        // ExtractPointCloudFromFloatDepth(im, intr, 1);
        auto planes_and_polygons = pl3d.ExtractPlanesAndPolygons(sparse_mesh, normals_mat);
    }
}

BENCHMARK_DEFINE_F(ImagesAndSparseMesh, BM_ExtractPlanesAndPolygonsFromMultipleNormalsOptimized)
(benchmark::State& st)
{
    std::vector<std::array<double, 3>> normals = {{-0.0192, -0.0199, -0.9996},
                                                  {0.0023, 0.0227, 0.9997},
                                                  {0.9995, -0.0213, -0.0227},
                                                  // { 0.9971,  0.0709, -0.0291},
                                                  {-0.0163, -0.9995, -0.0267},
                                                  // {0.0168, -0.2067,  0.9783},
                                                  {-0., 0.5988, 0.8009}};
    Polylidar3D pl3d(0.0, 0.1, 1000, 6, 0.03, 0.95, 0.90);
    const Matrix<double> normals_mat((double*)(normals.data()), st.range(0), 3);
    for (auto _ : st)
    {
        // ExtractPointCloudFromFloatDepth(im, intr, 1);
        auto planes_and_polygons = pl3d.ExtractPlanesAndPolygonsOptimized(sparse_mesh, normals_mat);
    }
}

BENCHMARK_DEFINE_F(DenseMesh, BM_ExtractPlanesAndPolygonsFromMultipleNormals)
(benchmark::State& st)
{

    Polylidar3D pl3d(0.0, 0.1, 1000, 6, 0.01, 0.95, 0.92);
    std::vector<std::array<double, 3>> normals = {{-0.0123, -0.0021, -0.9999},
                                                  {-0.2241, 0.9744, -0.0171},
                                                  {0.2207, -0.9752, 0.0144},
                                                  {-0.9612, -0.2756, 0.0129}};

    const Matrix<double> normals_mat((double*)(normals.data()), st.range(0), 3);
    for (auto _ : st)
    {
        auto planes_and_polygons = pl3d.ExtractPlanesAndPolygons(dense_mesh, normals_mat);
    }
}

BENCHMARK_DEFINE_F(DenseMesh, BM_ExtractPlanesAndPolygonsFromMultipleNormalsOptimized)
(benchmark::State& st)
{

    Polylidar3D pl3d(0.0, 0.1, 1000, 6, 0.01, 0.95, 0.92);
    std::vector<std::array<double, 3>> normals = {{-0.0123, -0.0021, -0.9999},
                                                  {-0.2241, 0.9744, -0.0171},
                                                  {0.2207, -0.9752, 0.0144},
                                                  {-0.9612, -0.2756, 0.0129}};

    const Matrix<double> normals_mat((double*)(normals.data()), st.range(0), 3);
    for (auto _ : st)
    {
        auto planes_and_polygons = pl3d.ExtractPlanesAndPolygonsOptimized(dense_mesh, normals_mat);
    }
}

BENCHMARK_REGISTER_F(ImagesAndSparseMesh, BM_Create_PointCloud)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(ImagesAndSparseMesh, BM_Create_TriMesh)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(ImagesAndSparseMesh, BM_ComputeTriangleNormals)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(ImagesAndSparseMesh, BM_CopyTriMesh)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(ImagesAndSparseMesh, BM_ExtractPlanesAndPolygons)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(ImagesAndSparseMesh, BM_ExtractPlanesAndPolygonsFromMultipleNormals)
    ->DenseRange(1, 4, 1)
    ->UseRealTime()
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(ImagesAndSparseMesh, BM_ExtractPlanesAndPolygonsFromMultipleNormalsOptimized)
    ->DenseRange(1, 4, 1)
    ->UseRealTime()
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(DenseMesh, BM_ExtractPlanesAndPolygonsFromMultipleNormals)
    ->DenseRange(1, 4, 1)
    ->UseRealTime()
    ->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(DenseMesh, BM_ExtractPlanesAndPolygonsFromMultipleNormalsOptimized)
    ->DenseRange(1, 4, 1)
    ->UseRealTime()
    ->Unit(benchmark::kMillisecond);
// BENCHMARK_REGISTER_F(Images, BM_ExtractPlanesAndPolygons)->UseRealTime()->Unit(benchmark::kMicrosecond);
