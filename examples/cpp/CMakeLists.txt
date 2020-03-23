
add_executable(polylidar-simple ${CMAKE_CURRENT_SOURCE_DIR}/simple.cpp)
target_link_libraries(polylidar-simple Polylidar::PL)