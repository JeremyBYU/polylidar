############ Start Version Parsing ##################

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" POLYLIDAR_VERSION_READ)
foreach(ver ${POLYLIDAR_VERSION_READ})
    if (ver MATCHES "(MAJOR|MINOR|PATCH) +([^ ]+)$")
        set(POLYLIDAR_VERSION_${CMAKE_MATCH_1} "${CMAKE_MATCH_2}" CACHE INTERNAL "")
    endif()
endforeach()
string(CONCAT POLYLIDAR_VERSION
    "${POLYLIDAR_VERSION_MAJOR}"
    ".${POLYLIDAR_VERSION_MINOR}"
    ".${POLYLIDAR_VERSION_PATCH}"
)

set(PROJECT_VERSION "${POLYLIDAR_VERSION}")

message("Version is ${PROJECT_VERSION}")

############# End Version Parsing ##################


add_subdirectory(Polylidar)

if (PL_BUILD_PYMODULE)
    add_subdirectory(Python)
endif ()
