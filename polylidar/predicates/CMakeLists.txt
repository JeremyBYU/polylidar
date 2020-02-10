cmake_minimum_required(VERSION 3.10.0)
project(predicates)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14")

set(LIB_TYPE STATIC)
set (predicates_SRCS
${CMAKE_CURRENT_SOURCE_DIR}/constants.c
${CMAKE_CURRENT_SOURCE_DIR}/predicates.c
${CMAKE_CURRENT_SOURCE_DIR}/random.c
${CMAKE_CURRENT_SOURCE_DIR}/printing.c
)

set(predicates_HDRS
${CMAKE_CURRENT_SOURCE_DIR}/constants.h
${CMAKE_CURRENT_SOURCE_DIR}/predicates.h
${CMAKE_CURRENT_SOURCE_DIR}/predicates.hpp
${CMAKE_CURRENT_SOURCE_DIR}/predicatesDLLExport.h
)

add_library (predicates ${LIB_TYPE} ${predicates_SRCS} ${predicates_HDRS})
# set_property(TARGET predicates PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
