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

# After the following call, the library will be installed
FetchContent_MakeAvailable(extern_parallel_hashmap extern_pybind11)