cmake_minimum_required(VERSION 3.11)
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
set(CMAKE_CXX_EXTENSIONS OFF)

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
# Options for Polylidar
option(PL_USE_ROBUST_PREDICATES "PL - Use Robust Geometric Predicates" OFF)
option(PL_BUILD_PYMODULE "PL - Build Python Module" ON)
option(PL_BUILD_TESTS "PL - Build Tests" ON)
option(PL_BUILD_BENCHMARKS "PL - Build Benchmarks" OFF)
option(PL_BUILD_EXAMPLES "PL - Build Examples" ON)
option(PL_WITH_OPENMP "PL - Build with OpenMP Support" ON)
option(PL_BUILD_WERROR "PL - Add Werror flag to build (turns warnings into errors)" OFF)

# Add any dependencies needed by our library
add_subdirectory("thirdparty")

# Build our library
add_subdirectory("src")

# Build examples if configured
if(PL_BUILD_EXAMPLES)
    add_subdirectory("examples")
endif()

# Build tests if configured
if(PL_BUILD_TESTS)
    add_subdirectory("tests")
endif()

# Build benchmarks if configured
if(PL_BUILD_BENCHMARKS)
    add_subdirectory("bench")
endif()
