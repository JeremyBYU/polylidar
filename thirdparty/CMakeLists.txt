include(FetchContent)
# Put any LIBRARY dependencies in here
FetchContent_Declare(
  extern_pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.4.3
)

FetchContent_Declare(
  extern_parallel_hashmap
  GIT_REPOSITORY https://github.com/greg7mdp/parallel-hashmap.git
  GIT_TAG        1.31
)

FetchContent_Declare(
  extern_eigen
  GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
  GIT_TAG        3.3.7
)

FetchContent_Declare(
  extern_marl
  GIT_REPOSITORY https://github.com/google/marl.git
  GIT_TAG        32af8bb50807f719818168eb165761e60e0090d5
)


# Build Robust Predicates if requested
if (PL_USE_ROBUST_PREDICATES)
    MESSAGE(STATUS "Building Robust Geometric Predicates")
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/predicates")
endif()


# After the following call, the library will be installed
FetchContent_MakeAvailable(extern_parallel_hashmap extern_pybind11 extern_eigen extern_marl)