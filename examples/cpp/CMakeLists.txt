add_executable(polylidar-simple ${CMAKE_CURRENT_SOURCE_DIR}/simple.cpp)
target_link_libraries(polylidar-simple Polylidar::PL)
target_include_directories(polylidar-simple PRIVATE "${PROJECT_SOURCE_DIR}/thirdparty/npy")


# Build Full Polylidar Example
if (PL_BUILD_FASTGA)
    message("Don't forget to tell cmake how to find Open3D, e.g, -DOpen3D_ROOT=C:/Users/Jerem/Documents/UMICH/Research/Open3D/build/open3d_install")
    find_package(Open3D REQUIRED)
    
    add_executable(polylidar-full ${CMAKE_CURRENT_SOURCE_DIR}/polylidar-full-example.cpp)
    target_link_libraries(polylidar-full PRIVATE Open3D::Open3D Polylidar::PL FastGA)
    target_include_directories(polylidar-full PRIVATE "${PROJECT_SOURCE_DIR}/thirdparty/npy")
endif()
