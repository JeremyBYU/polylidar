include(FetchContent)

FetchContent_Declare(
  extern_googlebenchmark
  GIT_REPOSITORY https://github.com/google/benchmark.git
  GIT_TAG        v1.5.0
)

# This module depend on having googletest and googlebenchmark installed
FetchContent_GetProperties(extern_googlebenchmark)
if(NOT extern_googlebenchmark_POPULATED)
  FetchContent_Populate(extern_googlebenchmark)
  SET(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Enable testing of the benchmark library.")
  add_subdirectory(${extern_googlebenchmark_SOURCE_DIR} ${extern_googlebenchmark_BINARY_DIR})
endif()

# Google Benchmark needs threads!
find_package(Threads REQUIRED)
find_package(OpenMP)
find_package(Open3D REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

message("Open3D ${Open3D_INCLUDE_DIRS}")

add_executable(run-bench ${CMAKE_CURRENT_SOURCE_DIR}/run-bench.cpp ${CMAKE_CURRENT_SOURCE_DIR}/bench_mesh.cpp
              )
              # ${CMAKE_CURRENT_SOURCE_DIR}/parallelism_test.cpp
target_link_libraries(run-bench PRIVATE Polylidar::PL benchmark Threads::Threads OpenMP::OpenMP_CXX Eigen3::Eigen ${Open3D_LIBRARIES})
target_link_directories(run-bench PRIVATE ${Open3D_LIBRARY_DIRS})
target_include_directories(run-bench PRIVATE ${Open3D_INCLUDE_DIRS})