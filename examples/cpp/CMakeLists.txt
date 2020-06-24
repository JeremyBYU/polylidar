add_executable(polylidar-simple ${CMAKE_CURRENT_SOURCE_DIR}/simple.cpp)
target_link_libraries(polylidar-simple Polylidar::PL)
target_include_directories(polylidar-simple PRIVATE "${PROJECT_SOURCE_DIR}/thirdparty/npy")

## NOT FOR PUBLIC ##
# find_package(Threads REQUIRED)
# find_package(Open3D REQUIRED)
# add_executable(polylidar-debug ${CMAKE_CURRENT_SOURCE_DIR}/debug.cpp)
# target_include_directories(polylidar-debug PRIVATE "${PROJECT_SOURCE_DIR}/thirdparty/npy")
# target_link_libraries(polylidar-debug PRIVATE Polylidar::PL benchmark Threads::Threads ${Open3D_LIBRARIES})
# target_link_directories(polylidar-debug PRIVATE ${Open3D_LIBRARY_DIRS})
# target_include_directories(polylidar-debug  PRIVATE ${Open3D_INCLUDE_DIRS})
