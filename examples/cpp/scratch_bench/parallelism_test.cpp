#include <iostream>
#include <sstream>
#include <vector>

#include <polylidar/polylidar.hpp>
#include <polylidar/util.hpp>
#include <benchmark/benchmark.h>
#include <omp.h>

#define VECTOR_SIZE_START 1024
#define VECTOR_SIZE_END 1048576


template<class T>
void Fill_Random(std::vector<T> &a)
{
    std::generate(a.begin(), a.end(), []() {
        return rand() % 10;
    });
}
template<class T>
void Simple_Add(std::vector<T> &a, std::vector<T> &b, std::vector<T> &c, bool parallel=false)
{
    int max_threads = parallel ? omp_get_max_threads() : 1;
    #pragma omp parallel for schedule(static) num_threads(max_threads)
    for (size_t i = 0; i < a.size(); i++)
    {
        c[i] = a[i] + b[i];
    }
}

void BM_Create_PointCloud(benchmark::State& state)
{
    size_t img_size = static_cast<size_t>(state.range(0));
    size_t num_pixels = img_size * img_size;
    // Create Image Data
    std::vector<float> im_(num_pixels);
    Fill_Random<float>(im_);
    polylidar::Matrix<float> im(im_.data(), img_size, img_size);
    // Create Intrinsics Data
    std::vector<double> intr_{307, 0, 0, 0, 307, 0, 204, 121, 1};
    polylidar::Matrix<double> intr(intr_.data(), 3, 3);

    // Create Extrinsics Data
    std::vector<double> extr_{1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0};
    polylidar::Matrix<double> extr(intr_.data(), 4, 4);

    for (auto _ : state) {
        ExtractPointCloudFromFloatDepth(im, intr, extr, 1);
    }
}

template<class T>
void BM_Simple_Add(benchmark::State& state) {
    size_t vector_size = static_cast<size_t>(state.range(1));
    bool parallel = static_cast<bool>(state.range(0));
    std::vector<T> a(vector_size);
    std::vector<T> b(vector_size);
    std::vector<T> c(vector_size);

    Fill_Random<T>(a);
    Fill_Random<T>(b);

    for (auto _ : state) {
        // This code gets timed
        Simple_Add<T>(a, b, c, parallel);
    }
}

// Register the function as a benchmark
BENCHMARK_TEMPLATE(BM_Simple_Add, double)->Ranges({{0, 1}, {VECTOR_SIZE_START,VECTOR_SIZE_END }})->UseRealTime();
BENCHMARK_TEMPLATE(BM_Simple_Add, float)->Ranges({{0, 1}, {VECTOR_SIZE_START,VECTOR_SIZE_END }})->UseRealTime();
BENCHMARK(BM_Create_PointCloud)->DenseRange(100, 500, 100)->UseRealTime()->Unit(benchmark::kMicrosecond);

// Run the benchmark
BENCHMARK_MAIN();