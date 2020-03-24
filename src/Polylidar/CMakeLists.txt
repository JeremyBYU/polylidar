

set(POLYLIDAR_SRC "${CMAKE_CURRENT_SOURCE_DIR}")
set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set(POLYLIDAR_INCLUDE_DIR "${INCLUDE_DIR}/Polylidar")
set(POLYLIDAR_SOURCE_FILES ${POLYLIDAR_SRC}/Polylidar.cpp ${POLYLIDAR_SRC}/MeshHelper.cpp ${POLYLIDAR_SRC}/Delaunator.cpp ${POLYLIDAR_SRC}/Core.cpp)
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

# OPTION Build with Robust Predicates
if (PL_USE_ROBUST_PREDICATES)
    MESSAGE(STATUS "Building with Robust Geometric Predicates")
    # add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/polylidar/predicates")
    # include_directories("${CMAKE_CURRENT_SOURCE_DIR}/polylidar/predicates")
    # target_link_libraries(polylidar predicates)
    # add_definitions(-DPL_USE_ROBUST_PREDICATES)
    # set_property(TARGET polylidar PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    # SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -flto")
endif()

# OPTON Build with OpenMP Support
if (WITH_OPENMP)
  find_package(OpenMP)
  target_link_libraries(${PROJECT_NAME}  PUBLIC OpenMP::OpenMP_CXX)
endif()

# Build with FastGA Support
# add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/GaussianAccumulator")
# target_link_libraries(polylidar PRIVATE FastGA::GA)

# Set Header and Properties of Polylidar Library
get_target_property(phmap_include_dirs phmap INTERFACE_INCLUDE_DIRECTORIES)
message("${phmap_include_dirs}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${phmap_include_dirs})
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_DIR})
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${POLYLIDAR_HEADER_FILES}")
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION} POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${INCLUDE_DIR}>
$<INSTALL_INTERFACE:include>
PRIVATE src)
# set_target_properties(polylidar PROPERTIES VERSION ${PROJECT_VERSION})

