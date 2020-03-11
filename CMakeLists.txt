cmake_minimum_required(VERSION 3.10.0)
include(CMakePrintHelpers)

project(polylidar)

# include(CheckIPOSupported)
# check_ipo_supported(RESULT supported OUTPUT error)
set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# Project Version and File
set(PROJECT_VERSION 0.0.8)
set(POLYLIDAR_SRC "${PROJECT_SOURCE_DIR}/polylidar")
set(POLYLIDAR_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/polylidar/include")
set(POLYLIDAR_EXAMPLES_CPP_DIR "${PROJECT_SOURCE_DIR}/examples/cpp")
set(POLYLIDAR_BENCH_CPP_DIR "${PROJECT_SOURCE_DIR}/bench")
set(POLYLIDAR_SOURCE_FILES ${POLYLIDAR_SRC}/polylidar.cpp ${POLYLIDAR_SRC}/helper.cpp ${POLYLIDAR_SRC}/delaunator.cpp ${POLYLIDAR_SRC}/MeshHelper.cpp)
set(POLYLIDAR_HEADER_FILES ${POLYLIDAR_INCLUDE_DIR}/polylidar/polylidar.hpp ${POLYLIDAR_INCLUDE_DIR}/polylidar/util.hpp 
${POLYLIDAR_INCLUDE_DIR}/polylidar/Mesh/MeshHelper.hpp ${POLYLIDAR_INCLUDE_DIR}/polylidar/helper.hpp ${POLYLIDAR_INCLUDE_DIR}/delaunator.hpp)
# Options for CMake
option(PL_USE_ROBUST_PREDICATES "Use Robust Geometric Predicates" OFF)
option(WITH_PROFILE "Enable Profiling" ON)
option(WITH_BENCHMARKS "Build Benchmarking" ON)
option(WITH_OPENMP "Build with OpenMP Support" ON)
option(WITH_FAST_GA "Build with FastGA" ON)
# CMake Flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native -march=native")

set(LIB_TYPE STATIC)
add_library(polylidar ${LIB_TYPE} ${POLYLIDAR_SOURCE_FILES})

# Enable warnings and error on them
set(MAXIMAL_DESIRED_WARNINGS "-Wall -Wextra -Wconversion -Wunreachable-code -Wuninitialized -pedantic-errors -Wold-style-cast -Wno-error=unused-variable -Wshadow -Wfloat-equal -Weffc++")
set(MINIMAL_DESIRED_WARNINGS "-Wall -Wextra -pedantic-errors")
separate_arguments(MAXIMAL_DESIRED_WARNINGS)
separate_arguments(MINIMAL_DESIRED_WARNINGS)
target_compile_options(polylidar PRIVATE
     $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:
          ${MINIMAL_DESIRED_WARNINGS}>
     $<$<CXX_COMPILER_ID:MSVC>:
          /W4>)

# OPTION Build with Robust Predicates
if (PL_USE_ROBUST_PREDICATES)
    MESSAGE(STATUS "Building with Robust Geometric Predicates")
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/polylidar/predicates")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/polylidar/predicates")
    target_link_libraries(polylidar predicates)
    add_definitions(-DPL_USE_ROBUST_PREDICATES)
    # set_property(TARGET polylidar PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    # SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
endif()

# OPTON Build with OpenMP Support
if (WITH_OPENMP)
  find_package(OpenMP)
  target_link_libraries(polylidar PUBLIC OpenMP::OpenMP_CXX)
endif()

# OPTON Build with OpenMP Support
if (WITH_FAST_GA)
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/GaussianAccumulator")
  target_link_libraries(polylidar PRIVATE FastGA::GA)
endif()

# Set Header and Properties of Polylidar Library
set_target_properties(polylidar PROPERTIES PUBLIC_HEADER "${POLYLIDAR_HEADER_FILES}")
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${POLYLIDAR_INCLUDE_DIR}>
$<INSTALL_INTERFACE:include>
PRIVATE src)
set_target_properties(polylidar PROPERTIES VERSION ${PROJECT_VERSION})
# set_target_properties(polylidar PROPERTIES PUBLIC_HEADER ${POLYLIDAR_HEADER_FILES})

# Build Simple Example
add_executable(polylidar-simple ${POLYLIDAR_EXAMPLES_CPP_DIR}/simple.cpp)
target_link_libraries(polylidar-simple polylidar)
target_include_directories(polylidar-simple PUBLIC ${POLYLIDAR_INCLUDE_DIR})

# OPTION Build Profiing Binary
if (WITH_PROFILE)
    MESSAGE(STATUS "Building with Profiling Enabled")
    set (Gperftools_DIR "${CMAKE_CURRENT_LIST_DIR}/cmake/")
    find_package(Gperftools REQUIRED)

    add_executable(polylidar-profile ${POLYLIDAR_EXAMPLES_CPP_DIR}/profile.cpp)
    target_link_libraries(polylidar-profile polylidar)
    target_include_directories(polylidar-profile PUBLIC ${POLYLIDAR_INCLUDE_DIR})
    target_link_libraries(polylidar-profile ${GPERFTOOLS_PROFILER})
endif()

# OPTION Build Benchmarks Binary
if (WITH_BENCHMARKS)
    MESSAGE(STATUS "Building bechmarks binary")
    # add_subdirectory_if_not_exists(gtest_main "thirdparty/googletest")
    add_subdirectory_if_not_exists(benchmark::benchmark "thirdparty/benchmark")
    # add_subdirectory("thirdparty/googletest")
    # add_subdirectory("thirdparty/benchmark")
    find_package(Open3D REQUIRED)
    set(BENCH_DEPENDENCIES benchmark::benchmark OpenMP::OpenMP_CXX polylidar ${Open3D_LIBRARIES})

    add_executable(parallel-test ${POLYLIDAR_BENCH_CPP_DIR}/parallelism_test.cpp)
    target_link_libraries(parallel-test ${BENCH_DEPENDENCIES})

    add_executable(bench-mesh ${POLYLIDAR_BENCH_CPP_DIR}/bench_mesh.cpp)
    target_include_directories(bench-mesh PRIVATE ${Open3D_INCLUDE_DIRS})
    target_link_libraries(bench-mesh ${BENCH_DEPENDENCIES})
    # Build Mesh Example
    add_executable(example-mesh ${POLYLIDAR_EXAMPLES_CPP_DIR}/example_mesh.cpp)
    target_link_libraries(example-mesh ${BENCH_DEPENDENCIES})
    target_include_directories(example-mesh PUBLIC ${POLYLIDAR_INCLUDE_DIR} ${Open3D_INCLUDE_DIRS})

    
endif()

