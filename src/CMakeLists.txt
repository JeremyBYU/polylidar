############# Start Build Options ###################

# Configure optimization
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(OPTIMIZATION_FLAGS "-O0 -DDEBUG -g")
    message("-- Configuring debug build")
else()
    set(OPTIMIZATION_FLAGS "-O3 -DNDEBUG -g")
    message("-- Configuring release build")
endif()

# Enable warnings, will set to compile with src/CMakeLists.txt
set(DESIRED_WARNINGS "-Wall -Wextra -Wconversion -Wunreachable-code -Wuninitialized -pedantic-errors -Wold-style-cast -Wno-error=unused-variable -Wshadow -Wfloat-equal -Weffc++")
set(MINIMAL_DESIRED_WARNINGS "-Wall -Wextra -pedantic-errors")
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(DESIRED_WARNINGS "${DESIRED_WARNINGS} -Wmost")
endif()
separate_arguments(DESIRED_WARNINGS)
separate_arguments(MINIMAL_DESIRED_WARNINGS)

if (OPF_BUILD_WERROR)
    set(DESIRED_WARNINGS "${DESIRED_WARNINGS} -Werror")
    set(MINIMAL_DESIRED_WARNINGS "${MINIMAL_DESIRED_WARNINGS} -Werror")
endif()

# Set GLOBAL CMAKE_CXX_FLAGS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPTIMIZATION_FLAGS}")

############# End Build Options #####################


############ Start Version Parsing ##################

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" POLYIDAR_VERSION_READ)
foreach(ver ${POLYIDAR_VERSION_READ})
    if (ver MATCHES "POLYLIDAR_VERSION_(MAJOR|MINOR|PATCH) +([^ ]+)$")
        set(POLYLIDAR_VERSION_${CMAKE_MATCH_1} "${CMAKE_MATCH_2}" CACHE INTERNAL "")
    endif()
endforeach()
string(CONCAT POLYLIDAR_VERSION
    "${POLYLIDAR_VERSION_MAJOR}"
    ".${POLYLIDAR_VERSION_MINOR}"
    ".${POLYLIDAR_VERSION_PATCH}"
)

set(PROJECT_VERSION "${POLYLIDAR_VERSION}")

############# End Version Parsing ##################

message("Version is ${POLYLIDAR_VERSION}")

add_subdirectory(Polylidar)
if (PL_BUILD_PYMODULE)
    add_subdirectory(Python)
endif ()
