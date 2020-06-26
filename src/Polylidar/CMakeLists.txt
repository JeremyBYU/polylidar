
configure_file("PLConfig.hpp.in" "PLConfig.hpp" @ONLY)

set(POLYLIDAR_SRC "${CMAKE_CURRENT_SOURCE_DIR}")
set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set(POLYLIDAR_INCLUDE_DIR "${INCLUDE_DIR}/Polylidar")
set(POLYLIDAR_SOURCE_FILES ${POLYLIDAR_SRC}/Polylidar.cpp ${POLYLIDAR_SRC}/MeshHelper.cpp ${POLYLIDAR_SRC}/MeshFilter.cpp 
                            ${POLYLIDAR_SRC}/Delaunator.cpp ${POLYLIDAR_SRC}/Core.cpp ${POLYLIDAR_SRC}/PLConfig.cpp)
set(POLYLIDAR_HEADER_FILES ${POLYLIDAR_INCLUDE_DIR}/Polylidar.hpp ${POLYLIDAR_INCLUDE_DIR}/Types.hpp 
                            ${POLYLIDAR_INCLUDE_DIR}/Utility.hpp ${POLYLIDAR_INCLUDE_DIR}/Mesh/MeshHelper.hpp
                            ${POLYLIDAR_INCLUDE_DIR}/Delaunator/Delaunator.hpp ${POLYLIDAR_INCLUDE_DIR}/Core.hpp)


set(LIB_TYPE STATIC)
add_library(${PROJECT_NAME} ${LIB_TYPE} ${POLYLIDAR_SOURCE_FILES})
add_library(Polylidar::PL ALIAS ${PROJECT_NAME})

### Enable warnings for ONLY this target. Error on them
target_compile_options(${PROJECT_NAME}  PRIVATE
$<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:
${DESIRED_WARNINGS}>
$<$<CXX_COMPILER_ID:MSVC>:
/W4>)

# To include the PLConfig.hpp file
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Link with Eigen
# Method to hide warnings from eigen using SYSTEM
get_target_property(eigen_include_dirs eigen INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${eigen_include_dirs})
# -pedantic-errors marl has issues with this
target_link_libraries_system_public(${PROJECT_NAME} marl)

# Fast Exponential
set(FASTEXP_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/thirdparty")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${FASTEXP_INCLUDE_DIR})

# Fast PHMAP
get_target_property(phmap_include_dirs phmap INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${phmap_include_dirs})

# OPTION Build with OpenMP Support
if (PL_WITH_OPENMP)
  find_package(OpenMP)
  target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

# OPTION Build with Robust Predicates
if (PL_USE_ROBUST_PREDICATES)
  MESSAGE(STATUS "Building with Robust Geometric Predicates")
  target_link_libraries(${PROJECT_NAME} PRIVATE PL_Predicates)
  target_compile_definitions(${PROJECT_NAME} PRIVATE PL_USE_ROBUST_PREDICATES)
endif()

# Build with FastGA Support
# add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/GaussianAccumulator")
# target_link_libraries(polylidar PRIVATE FastGA::GA)


# Set Properties
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${POLYLIDAR_HEADER_FILES}")
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION} POSITION_INDEPENDENT_CODE ON)
# Set Public and Private Include Directories
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIR})
# TODO - This is something that I still do not fully understand, Build vs Install?
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

