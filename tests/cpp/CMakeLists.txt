include(FetchContent)
FetchContent_Declare(
  extern_doctest
  GIT_REPOSITORY https://github.com/onqtam/doctest.git
  GIT_TAG        2.3.7
)

# This module depend on having doctest installed
FetchContent_MakeAvailable(extern_doctest)

add_executable(run-tests ${CMAKE_CURRENT_SOURCE_DIR}/run-tests.cpp)
target_link_libraries(run-tests PRIVATE doctest::doctest Polylidar::PL)
