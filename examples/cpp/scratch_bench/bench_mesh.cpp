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

// void BM_Create_PointCloud(benchmark::State& state)
// {
//     size_t img_size = static_cast<size_t>(state.range(0));
//     size_t num_pixels = img_size * img_size;
//     // Create Image Data
//     std::vector<float> im_(num_pixels);
//     Fill_Random<float>(im_);
//     polylidar::Matrix<float> im(im_.data(), img_size, img_size);
//     // Create Intrinsics Data
//     std::vector<double> intr_{307, 0, 0, 0, 307, 0, 204, 121, 1};
//     polylidar::Matrix<double> intr(intr_.data(), 3, 3);

//     for (auto _ : state) {
//         ExtractPointCloudFromFloatDepth(im, intr, 1);
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
        auto points = ExtractPointCloudFromFloatDepth(im, intr, extr, 1);
    }
}

BENCHMARK_DEFINE_F(Images, BM_Create_PointCloud_Eigen)
(benchmark::State& st)
{
    auto im_ptr = image_float->PointerAt<float>(0, 0);
    image_float->height_;
    polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
    Eigen::Map<Eigen::Matrix3d> intrin_e(intr_.data(),3,3);
    // Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(intr_.data());
    Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
    Eigen::Matrix2d m;
    m << cos(M_PI_2),-sin(M_PI_2),
        sin(M_PI_2),cos(M_PI_2);
    extrinsic.block<2,2>(0,0) = m;
    for (auto _ : st)
    {
        auto points = ExtractPointCloudFromFloatDepth2(im, intr, extr, 1);
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
        auto triMesh = polylidar::ExtractTriMeshFromFloatDepth(im, intr, extr, 1, false);
    }
}

BENCHMARK_DEFINE_F(Images, BM_ComputeTriangleNormals)
(benchmark::State& st)
{
    auto im_ptr = image_float->PointerAt<float>(0, 0);
    image_float->height_;
    polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
    auto triMesh = polylidar::ExtractTriMeshFromFloatDepth(im, intr, extr, 1, false);
    
    for (auto _ : st)
    {
        std::vector<double> triangle_normals;
        polylidar::ComputeTriangleNormals(triMesh.coords, triMesh.triangles, triangle_normals);
    }
}

BENCHMARK_DEFINE_F(Images, BM_ComputeTriangleNormals_Eigen)
(benchmark::State& st)
{
    auto im_ptr = image_float->PointerAt<float>(0, 0);
    image_float->height_;
    polylidar::Matrix<float> im(im_ptr, image_float->width_, image_float->height_);
    auto triMesh = polylidar::ExtractTriMeshFromFloatDepth(im, intr, extr, 1, false);
    
    for (auto _ : st)
    {
        std::vector<double> triangle_normals;
        polylidar::ComputeTriangleNormals2(triMesh.coords, triMesh.triangles, triangle_normals);
    }
}


BENCHMARK_REGISTER_F(Images, BM_Create_PointCloud)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_Create_PointCloud_Eigen)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_Create_TriMesh)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_ComputeTriangleNormals)->UseRealTime()->Unit(benchmark::kMicrosecond);
BENCHMARK_REGISTER_F(Images, BM_ComputeTriangleNormals_Eigen)->UseRealTime()->Unit(benchmark::kMicrosecond);
// BENCHMARK(BM_Create_PointCloud)->DenseRange(100, 500, 100)->UseRealTime()->Unit(benchmark::kMicrosecond);
// Run the benchmark
BENCHMARK_MAIN();