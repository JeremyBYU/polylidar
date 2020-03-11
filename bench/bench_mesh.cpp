#include <iostream>
#include <sstream>
#include <vector>

#include <polylidar/polylidar.hpp>
#include <polylidar/util.hpp>
#include <benchmark/benchmark.h>
#include <Open3D/IO/ClassIO/ImageIO.h>
#include <Open3D/IO/ClassIO/TriangleMeshIO.h>
#include <Open3D/Geometry/Image.h>
#include <Open3D/Geometry/TriangleMesh.h>
#include <omp.h>

const std::string DEFAULT_DEPTH_IMAGE = "./tests/fixtures/realsense/depth/00000003_1580908154939.png";
const std::string SPARSE_MESH = "./tests/fixtures/meshes/sparse_basement.ply";

namespace o3d = open3d;
using TriMesh = polylidar::MeshHelper::TriMesh;
using namespace polylidar;
class Images : public benchmark::Fixture
{
public:
    std::shared_ptr<o3d::geometry::Image> image_z16;
    std::shared_ptr<o3d::geometry::Image> image_float;
    std::vector<double> intr_{307, 0, 0, 0, 307, 0, 204, 121, 1};
    polylidar::Matrix<double> intr;
    std::vector<double> extr_{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    polylidar::Matrix<double> extr;
    // Mesh Files from .ply files
    std::shared_ptr<o3d::geometry::TriangleMesh> sparse_mesh_o3d;
    TriMesh sparse_mesh;
    Eigen::Matrix3d mesh_rotation;
    void SetUp(const ::benchmark::State &state)
    {
        o3d::geometry::Image image_z16_;
        auto success = o3d::io::ReadImage(DEFAULT_DEPTH_IMAGE, image_z16_);
        image_z16 = std::make_shared<o3d::geometry::Image>(image_z16_);
        image_float =  image_z16->ConvertDepthToFloatImage();
        intr.ptr = intr_.data(),
        intr.cols =  3;
        intr.rows = 3;
        extr = polylidar::Matrix<double>(intr_.data(), 4, 4);

        // Rotation for mesh, camera axis to global frame
        mesh_rotation << 1.0, 0.0, 0.0,
                         0.0, 0.0, 1.0,
                         0.0, -1.0, 0.0;
        // Sparse Mesh In O3D Format
        sparse_mesh_o3d = o3d::io::CreateMeshFromFile(SPARSE_MESH);
        sparse_mesh_o3d->Rotate(mesh_rotation);

        // test << 
        // sparse_mesh_o3d->Rotate();
        // Sparse Mesh In HalfEdgeTriangulation
        auto triangles_ptr = reinterpret_cast<int*>(sparse_mesh_o3d->triangles_.data());
        auto num_triangles = sparse_mesh_o3d->triangles_.size();
        auto vertices_ptr = reinterpret_cast<double*>(sparse_mesh_o3d->vertices_.data());
        auto num_vertices = sparse_mesh_o3d->vertices_.size();
        sparse_mesh = polylidar::MeshHelper::CreateTriMeshCopy(vertices_ptr, num_vertices, triangles_ptr, num_triangles);
    }
};


BENCHMARK_DEFINE_F(Images, BM_Create_PointCloud)
(benchmark::State& st)
{
    auto im_ptr = image_float->PointerAt<float>(0, 0);
    image_float->height_;
    polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
    for (auto _ : st)
    {
        auto points = MeshHelper::ExtractPointCloudFromFloatDepth(im, intr, extr, 2);
    }
}

BENCHMARK_DEFINE_F(Images, BM_Create_TriMesh)
(benchmark::State& st)
{
    auto im_ptr = image_float->PointerAt<float>(0, 0);
    image_float->height_;
    polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
    for (auto _ : st)
    {
        // ExtractPointCloudFromFloatDepth(im, intr, 1);
        auto triMesh = MeshHelper::ExtractTriMeshFromFloatDepth(im, intr, extr, 2, false);
    }
}

BENCHMARK_DEFINE_F(Images, BM_ComputeTriangleNormals)
(benchmark::State& st)
{
    auto im_ptr = image_float->PointerAt<float>(0, 0);
    image_float->height_;
    polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
    auto triMesh = MeshHelper::ExtractTriMeshFromFloatDepth(im, intr, extr,2, false);
    
    for (auto _ : st)
    {
        std::vector<double> triangle_normals;
        polylidar::MeshHelper::ComputeTriangleNormals(triMesh.coords, triMesh.triangles, triangle_normals);
    }
}

BENCHMARK_DEFINE_F(Images, BM_ExtractPlanesTriMesh)
(benchmark::State& st)
{
    polylidar::Config config{3, 0.0, 0.0, 0.10, 1000, 6, 0.0, 0.03, 0.95, 0.90, 1.0, {{0, 0, 1}}};
    // std::cout << sparse_mesh.vertices.size() / 3 << std::endl;
    for (auto _ : st)
    {
        // ExtractPointCloudFromFloatDepth(im, intr, 1);
        auto planes = polylidar::extractPlanesSet(sparse_mesh, sparse_mesh.coords, config);
    }
}

BENCHMARK_DEFINE_F(Images, BM_ExtractPlanesAndPolygons)
(benchmark::State& st)
{
    // dict(alpha=0.0, lmax=0.10, minTriangles=1000,
    //                         zThresh=0.03, normThresh=0.95, normThreshMin=0.90, minHoleVertices=6)
    polylidar::Config config{3, 0.0, 0.0, 0.10, 1000, 6, 0.0, 0.03, 0.95, 0.90, 1.0, {{0, 0, 1}}};
    for (auto _ : st)
    {
        std::vector<std::vector<size_t>> planes;
        std::vector<polylidar::Polygon> polygons;
        // ExtractPointCloudFromFloatDepth(im, intr, 1);
        std::tie(planes, polygons) = polylidar::ExtractPlanesAndPolygonsFromMesh(sparse_mesh, config);
    }
}

BENCHMARK_DEFINE_F(Images, BM_ExtractPolygonsFromMultipleNormals_Sparse)
(benchmark::State& st)
{
    // dict(alpha=0.0, lmax=0.10, minTriangles=1000,
    //                         zThresh=0.03, normThresh=0.95, normThreshMin=0.90, minHoleVertices=6)
    polylidar::Config config{3, 0.0, 0.0, 0.10, 1000, 6, 0.0, 0.03, 0.95, 0.90, 1.0, {{0, 0, 1}}};
    std::vector<std::array<double, 3>> normals =     {{-0.0192, -0.0199, -0.9996},
    { 0.0023,  0.0227,  0.9997},
    { 0.9995, -0.0213, -0.0227},
    // { 0.9971,  0.0709, -0.0291},
    {-0.0163, -0.9995, -0.0267},
    // {0.0168, -0.2067,  0.9783},
    {-0.,      0.5988,  0.8009}};
    const polylidar::Matrix<double> normals_mat((double *)(normals.data()), st.range(0), 3);
    for (auto _ : st)
    {
        // ExtractPointCloudFromFloatDepth(im, intr, 1);
        auto polygons = polylidar::ExtractPolygonsFromMesh(sparse_mesh, normals_mat, config);
    }
}



BENCHMARK_REGISTER_F(Images, BM_Create_PointCloud)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_Create_TriMesh)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_ComputeTriangleNormals)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_ExtractPlanesTriMesh)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_ExtractPlanesAndPolygons)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_ExtractPolygonsFromMultipleNormals_Sparse)->DenseRange(1, 4, 1)->UseRealTime()->Unit(benchmark::kMicrosecond);
// Run the benchmark
BENCHMARK_MAIN();