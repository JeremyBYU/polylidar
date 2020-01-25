cmake_minimum_required(VERSION 3.10.0)
project(polylidar)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14 -Wno-narrowing -Wno-register -fPIC -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native -march=native")
set(CMAKE_BUILD_TYPE "Release")

set(PROJECT_VERSION 0.0.8)
set(POLYLIDAR_SRC "${PROJECT_SOURCE_DIR}/polylidar")
set(POLYLIDAR_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/polylidar/include")
set(POLYLIDAR_EXAMPLES_CPP_DIR "${PROJECT_SOURCE_DIR}/examples/cpp")


set(POLYLIDAR_SOURCE_FILES ${POLYLIDAR_SRC}/polylidar.cpp ${POLYLIDAR_SRC}/helper.cpp ${POLYLIDAR_SRC}/delaunator.cpp)
set(POLYLIDAR_HEADER_FILES ${POLYLIDAR_INCLUDE_DIR}/polylidar/polylidar.hpp ${POLYLIDAR_INCLUDE_DIR}/polylidar/util.hpp ${POLYLIDAR_INCLUDE_DIR}/delaunator.hpp)

# message ("POLYLIDAR_SOURCE_FILES = ${POLYLIDAR_SOURCE_FILES}")

add_library(polylidar SHARED
${POLYLIDAR_SOURCE_FILES}
)

set_target_properties(polylidar PROPERTIES PUBLIC_HEADER ${POLYLIDAR_HEADER_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${POLYLIDAR_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

set_target_properties(polylidar PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(polylidar PROPERTIES PUBLIC_HEADER ${POLYLIDAR_HEADER_FILES})

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

# Add the application sources to the target
add_executable(polylidar-simple ${POLYLIDAR_EXAMPLES_CPP_DIR}/simple.cpp)
target_link_libraries(polylidar-simple polylidar)
target_include_directories(polylidar-simple PUBLIC ${POLYLIDAR_INCLUDE_DIR})

