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
    std::vector<double> extr_{1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0};
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


// void BM_FloatComparision(benchmark::State& st)
// {
//     std::array<double, 10000> test;
//     std::array<int, 10000> test1;
//     test[100] = 0.0;
//     for (auto _ : st)
//     {
//         int counter =0;
//         for (auto && item: test)
//         {
//             if(item == 0.0)
//             {
//                 test1[counter] = 1;
//             }
//             counter++;
//         }
//         benchmark::DoNotOptimize(test1.data());
//     }
// }

// void BM_NanComparision(benchmark::State& st)
// {
//     std::array<double, 10000> test;
//     std::array<int, 10000> test1;
//     test[100] = std::numeric_limits<double>::quiet_NaN();
//     for (auto _ : st)
//     {
//         int counter =0;
//         for (auto && item: test)
//         {
//             if(std::isnan(item))
//             {
//                 test1[counter] = 1;
//             }
//             counter++;
//         }
//         benchmark::DoNotOptimize(test1.data());
//     }
// }

// void BM_InfComparision(benchmark::State& st)
// {
//     std::array<double, 10000> test;
//     std::array<int, 10000> test1;
//     test[100] = std::numeric_limits<double>::infinity();
//     for (auto _ : st)
//     {
//         int counter =0;
//         for (auto && item: test)
//         {
//             if(std::isinf(item))
//             {
//                 test1[counter] = 1;
//             }
//             counter++;
//         }
//         benchmark::DoNotOptimize(test1.data());
//     }
// }

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

// BENCHMARK_DEFINE_F(Images, BM_Create_PointCloud3)
// (benchmark::State& st)
// {
//     auto im_ptr = image_float->PointerAt<float>(0, 0);
//     image_float->height_;
//     polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
//     for (auto _ : st)
//     {
//         std::vector<double> points;
//         std::vector<bool> valid;
//         std::tie(points, valid) = ExtractPointCloudFromFloatDepth3(im, intr, extr, 1);
//     }
// }



// BENCHMARK_DEFINE_F(Images, BM_Create_PointCloud_Eigen)
// (benchmark::State& st)
// {
//     auto im_ptr = image_float->PointerAt<float>(0, 0);
//     image_float->height_;
//     polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
//     Eigen::Map<Eigen::Matrix3d> intrin_e(intr_.data(),3,3);
//     // Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(intr_.data());
//     Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
//     Eigen::Matrix2d m;
//     m << cos(M_PI_2),-sin(M_PI_2),
//         sin(M_PI_2),cos(M_PI_2);
//     extrinsic.block<2,2>(0,0) = m;
//     for (auto _ : st)
//     {
//         auto points = ExtractPointCloudFromFloatDepth2(im, intr, extr, 2);
//     }
// }

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
// BENCHMARK_REGISTER_F(Images, BM_ComputeTriangleNormals_Eigen)->UseRealTime()->Unit(benchmark::kMicrosecond);
// BENCHMARK(BM_Create_PointCloud)->DenseRange(100, 500, 100)->UseRealTime()->Unit(benchmark::kMicrosecond);
// Run the benchmark
BENCHMARK_MAIN();