
add_executable(polylidar-simple ${CMAKE_CURRENT_SOURCE_DIR}/simple.cpp)
target_link_libraries(polylidar-simple Polylidar::PL)
target_include_directories(polylidar-simple PRIVATE "${PROJECT_SOURCE_DIR}/thirdparty/npy")


set(OLD_SOURCES "${PROJECT_SOURCE_DIR}/polylidar/polylidar.cpp" "${PROJECT_SOURCE_DIR}/polylidar/MeshHelper.cpp" "${PROJECT_SOURCE_DIR}/polylidar/delaunator.cpp" "${PROJECT_SOURCE_DIR}/polylidar/helper.cpp")

add_executable(polylidar-simple-old ${CMAKE_CURRENT_SOURCE_DIR}/simple_old.cpp ${OLD_SOURCES})
target_link_libraries(polylidar-simple-old )
target_include_directories(polylidar-simple-old PRIVATE "${PROJECT_SOURCE_DIR}/thirdparty/npy")
target_include_directories(polylidar-simple-old PRIVATE "${PROJECT_SOURCE_DIR}/polylidar/include")