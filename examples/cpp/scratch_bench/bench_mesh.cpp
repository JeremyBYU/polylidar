#include <iostream>
#include <sstream>
#include <vector>

#include <polylidar/polylidar.hpp>
#include <polylidar/util.hpp>
#include <benchmark/benchmark.h>
#include <Open3D/IO/ClassIO/ImageIO.h>
#include <Open3D/Geometry/Image.h>
#include <omp.h>
#include <Eigen/Core>

const std::string DEFAULT_DEPTH_IMAGE = "./tests/fixtures/realsense/depth/00000003_1580908154939.png";

namespace o3d = open3d;

class Images : public benchmark::Fixture
{
public:
    std::shared_ptr<o3d::geometry::Image> image_z16;
    std::shared_ptr<o3d::geometry::Image> image_float;
    std::vector<double> intr_{307, 0, 0, 0, 307, 0, 204, 121, 1};
    polylidar::Matrix<double> intr;
    std::vector<double> extr_{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    polylidar::Matrix<double> extr;
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
        auto points = ExtractPointCloudFromFloatDepth(im, intr, extr, 2);
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
        auto triMesh = polylidar::ExtractTriMeshFromFloatDepth(im, intr, extr, 2, false);
    }
}

BENCHMARK_DEFINE_F(Images, BM_ComputeTriangleNormals)
(benchmark::State& st)
{
    auto im_ptr = image_float->PointerAt<float>(0, 0);
    image_float->height_;
    polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
    auto triMesh = polylidar::ExtractTriMeshFromFloatDepth(im, intr, extr,2, false);
    
    for (auto _ : st)
    {
        std::vector<double> triangle_normals;
        polylidar::ComputeTriangleNormals(triMesh.coords, triMesh.triangles, triangle_normals);
    }
}

BENCHMARK_DEFINE_F(Images, BM_ExtractPlanesAndPolygons)
(benchmark::State& st)
{
    auto im_ptr = image_float->PointerAt<float>(0, 0);
    image_float->height_;
    polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
    delaunator::TriMesh triMesh = polylidar::ExtractTriMeshFromFloatDepth(im, intr, extr,2, false);
        // polylidar_kwargs = dict(alpha=0.0, lmax=0.10, minTriangles=100,
                            // zThresh=0.03, normThresh=0.99, normThreshMin=0.95, minHoleVertices=6)
    polylidar::Config config{3, 0.0, 0.0, 0.10, 100, 6, 0.0, 0.03, 0.99, 0.95, 1.0};
    for (auto _ : st)
    {
        std::vector<std::vector<size_t>> planes;
        std::vector<polylidar::Polygon> polygons;
        // ExtractPointCloudFromFloatDepth(im, intr, 1);
        std::tie(planes, polygons) = polylidar::ExtractPlanesAndPolygonsFromMesh(triMesh, config);
    }
}

BENCHMARK_DEFINE_F(Images, BM_ExtractPlanesTriMesh)
(benchmark::State& st)
{
    auto im_ptr = image_float->PointerAt<float>(0, 0);
    image_float->height_;
    polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
    delaunator::TriMesh triMesh = polylidar::ExtractTriMeshFromFloatDepth(im, intr, extr,2, false);
    // delaunator::HalfEdgeTriangulation triMesh2 = triMesh;
        // polylidar_kwargs = dict(alpha=0.0, lmax=0.10, minTriangles=100,
                            // zThresh=0.03, normThresh=0.99, normThreshMin=0.95, minHoleVertices=6)
    polylidar::Config config{3, 0.0, 0.0, 0.10, 100, 6, 0.0, 0.03, 0.99, 0.95, 1.0};
    for (auto _ : st)
    {
        // ExtractPointCloudFromFloatDepth(im, intr, 1);
        auto planes = polylidar::extractPlanesSet(triMesh, triMesh.coords, config);
    }
}

// BENCHMARK_DEFINE_F(Images, BM_ExtractPlanesHalfEdge)
// (benchmark::State& st)
// {
//     auto im_ptr = image_float->PointerAt<float>(0, 0);
//     image_float->height_;
//     polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
//     delaunator::HalfEdgeTriangulation triMesh = polylidar::ExtractTriMeshFromFloatDepth(im, intr, extr,2, false);
//     // delaunator::HalfEdgeTriangulation triMesh2 = triMesh;
//         // polylidar_kwargs = dict(alpha=0.0, lmax=0.10, minTriangles=100,
//                             // zThresh=0.03, normThresh=0.99, normThreshMin=0.95, minHoleVertices=6)
//     polylidar::Config config{3, 0.0, 0.0, 0.10, 100, 6, 0.0, 0.03, 0.99, 0.95, 1.0};
//     for (auto _ : st)
//     {
//         // ExtractPointCloudFromFloatDepth(im, intr, 1);
//         auto planes = polylidar::extractPlanesSet(triMesh, triMesh.coords, config);
//     }
// }

// BENCHMARK_DEFINE_F(Images, BM_ComputeTriangleNormals_Eigen)
// (benchmark::State& st)
// {
//     auto im_ptr = image_float->PointerAt<float>(0, 0);
//     image_float->height_;
//     polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
//     auto triMesh = polylidar::ExtractTriMeshFromFloatDepth(im, intr, extr, 2, false);
    
//     for (auto _ : st)
//     {
//         std::vector<double> triangle_normals;
//         polylidar::ComputeTriangleNormals2(triMesh.coords, triMesh.triangles, triangle_normals);
//     }
// }


// BENCHMARK(BM_FloatComparision)->UseRealTime()->Unit(benchmark::kNanosecond);
// BENCHMARK(BM_NanComparision)->UseRealTime()->Unit(benchmark::kNanosecond);
// BENCHMARK(BM_InfComparision)->UseRealTime()->Unit(benchmark::kNanosecond);
BENCHMARK_REGISTER_F(Images, BM_Create_PointCloud)->UseRealTime()->Unit(benchmark::kMicrosecond);
// BENCHMARK_REGISTER_F(Images, BM_Create_PointCloud3)->UseRealTime()->Unit(benchmark::kMicrosecond);
// BENCHMARK_REGISTER_F(Images, BM_Create_PointCloud_Eigen)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_Create_TriMesh)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_ComputeTriangleNormals)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_ExtractPlanesTriMesh)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_ExtractPlanesAndPolygons)->UseRealTime()->Unit(benchmark::kMicrosecond);
// BENCHMARK_REGISTER_F(Images, BM_ExtractPlanes2)->UseRealTime()->Unit(benchmark::kMicrosecond);
// BENCHMARK_REGISTER_F(Images, BM_ComputeTriangleNormals_Eigen)->UseRealTime()->Unit(benchmark::kMicrosecond);
// BENCHMARK(BM_Create_PointCloud)->DenseRange(100, 500, 100)->UseRealTime()->Unit(benchmark::kMicrosecond);
// Run the benchmark
BENCHMARK_MAIN();