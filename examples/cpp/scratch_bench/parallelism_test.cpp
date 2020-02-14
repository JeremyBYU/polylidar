#include <iostream>
#include <sstream>
#include <vector>

#include <benchmark/benchmark.h>
#include <omp.h>

#define VECTOR_SIZE_START 1024
#define VECTOR_SIZE_END 1048576

template<class T>
void Fill_Random(std::vector<T> &a)
{
    std::generate(a.begin(), a.end(), []() {
        return rand() % 100;
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

// Run the benchmark
BENCHMARK_MAIN();