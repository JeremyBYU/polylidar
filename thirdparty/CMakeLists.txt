include(FetchContent)
# Put any LIBRARY dependencies in here
FetchContent_Declare(
  extern_pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.4.3
  GIT_PROGRESS   TRUE
)

FetchContent_Declare(
  extern_parallel_hashmap
  GIT_REPOSITORY https://github.com/greg7mdp/parallel-hashmap.git
  GIT_TAG        1.31
  GIT_PROGRESS   TRUE
)

FetchContent_Declare(
  extern_eigen
  GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
  GIT_TAG        3.3.7
  GIT_PROGRESS   TRUE
)

FetchContent_Declare(
  extern_marl
  GIT_REPOSITORY https://github.com/google/marl.git
  GIT_TAG        32af8bb50807f719818168eb165761e60e0090d5
  GIT_PROGRESS   TRUE
)

FetchContent_Declare(
  extern_fastga
  GIT_REPOSITORY https://github.com/JeremyBYU/FastGaussianAccumulator
  GIT_TAG        origin/master
  GIT_PROGRESS   TRUE
)


# Build Robust Predicates if requested
if (PL_USE_ROBUST_PREDICATES)
    MESSAGE(STATUS "Building Robust Geometric Predicates")
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/predicates")
endif()

# After the following call, the library will be installed
FetchContent_MakeAvailable(extern_parallel_hashmap extern_pybind11 extern_eigen extern_marl)

# Build Robust Predicates if requested
if (PL_BUILD_FASTGA AND PL_BUILD_EXAMPLES)
    MESSAGE(STATUS "Building Examples with FastGA")
    # Add fastga but dont build examples and unit tests
    FetchContent_GetProperties(extern_fastga)
    if(NOT extern_fastga_POPULATED)
      FetchContent_Populate(extern_fastga)
      set(GA_BUILD_PYMODULE OFF CACHE BOOL "GA -Build Python Module" FORCE)
      set(GA_BUILD_TESTS OFF CACHE BOOL "GA - Build Tests" FORCE)
      set(GA_BUILD_BENCHMARKS OFF CACHE BOOL "GA - Build Benchmarks" FORCE)
      set(GA_BUILD_EXAMPLES OFF CACHE BOOL "GA - Build Examples" FORCE)
      add_subdirectory(${extern_fastga_SOURCE_DIR} ${extern_fastga_BINARY_DIR})
    endif()
endif()
