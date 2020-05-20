cmake_minimum_required(VERSION 3.10.0)
project(PL_Predicates)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14")

set(LIB_TYPE SHARED)
set (PL_predicates_SRCS
${CMAKE_CURRENT_SOURCE_DIR}/constants.c
${CMAKE_CURRENT_SOURCE_DIR}/predicates.c
${CMAKE_CURRENT_SOURCE_DIR}/random.c
${CMAKE_CURRENT_SOURCE_DIR}/printing.c
)

set(PL_predicates_HDRS
${CMAKE_CURRENT_SOURCE_DIR}/constants.h
${CMAKE_CURRENT_SOURCE_DIR}/predicates.h
${CMAKE_CURRENT_SOURCE_DIR}/predicates.hpp
${CMAKE_CURRENT_SOURCE_DIR}/predicatesDLLExport.h
)

set(PL_predicates_header_dir ${CMAKE_CURRENT_SOURCE_DIR})
add_library (PL_Predicates ${LIB_TYPE} ${PL_predicates_SRCS} ${PL_predicates_HDRS})

target_include_directories(PL_Predicates PUBLIC
  $<BUILD_INTERFACE:${PL_predicates_header_dir}>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)
# set_property(TARGET predicates PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
