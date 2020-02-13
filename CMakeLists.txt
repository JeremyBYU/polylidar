cmake_minimum_required(VERSION 3.10.0)
project(polylidar)

# include(CheckIPOSupported)
# check_ipo_supported(RESULT supported OUTPUT error)

set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native -march=native")

set(PROJECT_VERSION 0.0.8)
set(POLYLIDAR_SRC "${PROJECT_SOURCE_DIR}/polylidar")
set(POLYLIDAR_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/polylidar/include")
set(POLYLIDAR_EXAMPLES_CPP_DIR "${PROJECT_SOURCE_DIR}/examples/cpp")

set(POLYLIDAR_SOURCE_FILES ${POLYLIDAR_SRC}/polylidar.cpp ${POLYLIDAR_SRC}/helper.cpp ${POLYLIDAR_SRC}/delaunator.cpp)
set(POLYLIDAR_HEADER_FILES ${POLYLIDAR_INCLUDE_DIR}/polylidar/polylidar.hpp ${POLYLIDAR_INCLUDE_DIR}/polylidar/util.hpp ${POLYLIDAR_INCLUDE_DIR}/delaunator.hpp)

option(PL_USE_ROBUST_PREDICATES "Use Robust Geometric Predicates" OFF)
option(WITH_PROFILE "Enable Profiling" ON)

set(LIB_TYPE STATIC)

add_library(polylidar ${LIB_TYPE} ${POLYLIDAR_SOURCE_FILES})

if (PL_USE_ROBUST_PREDICATES)
    MESSAGE(STATUS "Building with Robust Geometric Predicates")
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/polylidar/predicates")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/polylidar/predicates")
    target_link_libraries(polylidar predicates)
    add_definitions(-DPL_USE_ROBUST_PREDICATES)
    # set_property(TARGET polylidar PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    # SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
endif()

find_package(OpenMP)
target_link_libraries(polylidar PUBLIC OpenMP::OpenMP_CXX)

set_target_properties(polylidar PROPERTIES PUBLIC_HEADER ${POLYLIDAR_HEADER_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${POLYLIDAR_INCLUDE_DIR}>
$<INSTALL_INTERFACE:include>
PRIVATE src)

set_target_properties(polylidar PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(polylidar PROPERTIES PUBLIC_HEADER ${POLYLIDAR_HEADER_FILES})

# Add the application sources to the target
add_executable(polylidar-simple ${POLYLIDAR_EXAMPLES_CPP_DIR}/simple.cpp)
target_link_libraries(polylidar-simple polylidar)
target_include_directories(polylidar-simple PUBLIC ${POLYLIDAR_INCLUDE_DIR})
if (WITH_PROFILE)
    MESSAGE(STATUS "Building with Profiling Enabled")
    set (Gperftools_DIR "${CMAKE_CURRENT_LIST_DIR}/cmake/")
    find_package(Gperftools REQUIRED)

    add_executable(polylidar-profile ${POLYLIDAR_EXAMPLES_CPP_DIR}/profile.cpp)
    target_link_libraries(polylidar-profile polylidar)
    target_include_directories(polylidar-profile PUBLIC ${POLYLIDAR_INCLUDE_DIR})
    target_link_libraries(polylidar-profile ${GPERFTOOLS_PROFILER})
endif()

