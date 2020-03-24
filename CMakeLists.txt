cmake_minimum_required(VERSION 3.11)
# set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
# Project Details
set(PROJECT_NAME "Polylidar")
set(PROJECT_VERSION 0.1.0)
set(PROJECT_EMAIL   "")
set(PROJECT_HOME    "")
set(PROJECT_DOCS    "")
set(PROJECT_CODE    "")
set(PROJECT_ISSUES  "")
set(PYPI_PACKAGE_NAME "polylidar")
# Set Project Properties
project(${PROJECT_NAME} VERSION ${PROJECT_VERSION}
DESCRIPTION "Fast Polygon Extraction from Point Clouds and Meshes"
LANGUAGES CXX)
# Set Global Properties
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(VERBOSE OFF)
if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Output Folders
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# CMake Functions
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Extras.cmake)

# Build Options
option(WERROR "Add -Werror flag to build (turns warnings into errors)" OFF)
option(WITH_PROFILE "Enable Profiling" ON)
# Options for Polylidar
option(PL_BUILD_PYMODULE "Build Python Module" ON)
option(PL_BUILD_DOCS "Build Documentation" OFF)
option(PL_USE_ROBUST_PREDICATES "Use Robust Geometric Predicates" OFF)

option(WITH_PROFILE "Enable Profiling" ON)
option(WITH_BENCHMARKS "Build Benchmarking" ON)
option(WITH_OPENMP "Build with OpenMP Support" ON)

# Configure optimization
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(OPTIMIZATION_FLAGS "-O0 -DDEBUG")
    message("-- Configuring debug build")
else()
    set(OPTIMIZATION_FLAGS "-O3 -DNDEBUG")
    message("-- Configuring release build")
endif()

# Enable extra warnings to adhere to https://github.com/mapbox/cpp/issues/37
set(DESIRED_WARNINGS "-Wall -Wextra -Wconversion -Wunreachable-code -Wuninitialized -pedantic-errors -Wold-style-cast -Wno-error=unused-variable -Wshadow -Wfloat-equal -Weffc++")
set(MINIMAL_DESIRED_WARNINGS "-Wall -Wextra -pedantic-errors")
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(DESIRED_WARNINGS "${DESIRED_WARNINGS} -Wmost")
endif()
separate_arguments(DESIRED_WARNINGS)
separate_arguments(MINIMAL_DESIRED_WARNINGS)

### Set GLOBAL CMAKE_CXX_FLAGS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPTIMIZATION_FLAGS}")
if (WERROR)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
endif()


# Add Dependencies
add_subdirectory_if_not_exists(phmap "thirdparty/parallel-hashmap")
if (PL_BUILD_PYMODULE)
    add_subdirectory_if_not_exists(pybind11::module "thirdparty/pybind11")
endif()


add_subdirectory("src")
add_subdirectory("examples")

if (PL_BUILD_DOCS)
    add_subdirectory("docs")
endif()
