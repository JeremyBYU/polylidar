// This defines the main entry point for the C++ Python Extension using Pybind11
#include "glue.hpp"

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::vector<size_t>);
PYBIND11_MAKE_OPAQUE(std::vector<double>);

using namespace pybind11::literals;

PYBIND11_MODULE(polylidar, m)
{

    // xt::import_numpy(); // MUST import numpy here
    m.doc() = R"pbdoc(
        Polylidar - Rapidly extract polygons from points clouds
        -----------------------

    )pbdoc";

    py::bind_vector<std::vector<std::size_t>>(m, "VectorInts", py::buffer_protocol());
    py::bind_vector<std::vector<double>>(m, "VectorDouble", py::buffer_protocol());

    py::class_<polylidar::Matrix>(m, "Matrix", py::buffer_protocol())
   .def_buffer([](polylidar::Matrix &m) -> py::buffer_info {
        return py::buffer_info(
            m.ptr,                               /* Pointer to buffer */
            sizeof(double),                          /* Size of one scalar */
            py::format_descriptor<double>::format(), /* Python struct-style format descriptor */
            2,                                      /* Number of dimensions */
            { m.rows, m.cols },                 /* Buffer dimensions */
            { sizeof(double) * m.cols,             /* Strides (in bytes) for each index */
              sizeof(double) }
        );
    });

    py::class_<delaunator::Delaunator>(m, "Delaunator")
        .def(py::init<py::array_t<double>>())
        .def("triangulate", &delaunator::Delaunator::triangulate)
        .def_readonly("triangles", &delaunator::Delaunator::triangles)
        .def_readonly("halfedges", &delaunator::Delaunator::halfedges)
        .def_readonly("hull_tri", &delaunator::Delaunator::hull_tri)
        .def_readonly("coords", &delaunator::Delaunator::coords);

    py::class_<polylidar::Polygon>(m, "Polygon")
        .def(py::init<>())
        .def_readonly("shell", &polylidar::Polygon::shell, py::return_value_policy::copy)
        // .def_readonly("holes", &polylidar::Polygon::holes, py::return_value_policy::copy)
        .def_property("holes", &polylidar::Polygon::getHoles, &polylidar::Polygon::setHoles);
    
    m.def("extractPlanesAndPolygons", &polylidar::extractPlanesAndPolygons, "Extracts planar meshes and polygons from a point cloud",
        "nparray"_a, "alpha"_a=DEFAULT_ALPHA, "xyThresh"_a=DEFAULT_XYTHRESH,
        "lmax"_a=DEFAULT_LMAX, "minTriangles"_a=DEFAULT_MINTRIANGLES, "minHoleVertices"_a=DEFAULT_MINHOLEVERTICES,
        "minBboxArea"_a=DEFAULT_MINBBOX, "zThresh"_a=DEFAULT_ZTHRESH,
        "normThresh"_a=DEFAULT_NORMTHRESH, "normThreshMin"_a=DEFAULT_NORMTHRESH_MIN,
        "allowedClass"_a=DEFAULT_ALLOWEDCLASS);

    m.def("extractPolygons", &polylidar::extractPolygons, "Extracts polygons from a point cloud",
        "nparray"_a, "alpha"_a=DEFAULT_ALPHA, "xyThresh"_a=DEFAULT_XYTHRESH,
        "lmax"_a=DEFAULT_LMAX, "minTriangles"_a=DEFAULT_MINTRIANGLES, "minHoleVertices"_a=DEFAULT_MINHOLEVERTICES,
        "minBboxArea"_a=DEFAULT_MINBBOX, "zThresh"_a=DEFAULT_ZTHRESH,
        "normThresh"_a=DEFAULT_NORMTHRESH, "normThreshMin"_a=DEFAULT_NORMTHRESH_MIN,
        "allowedClass"_a=DEFAULT_ALLOWEDCLASS);

    m.def("extractPolygonsAndTimings", &polylidar::extractPolygonsAndTimings, "Extracts polygons from a point cloud and returns detailed timings of triangulation, mesh extraction, and polygon extraction", 
        "nparray"_a, "alpha"_a=DEFAULT_ALPHA, "xyThresh"_a=DEFAULT_XYTHRESH,
        "lmax"_a=DEFAULT_LMAX, "minTriangles"_a=DEFAULT_MINTRIANGLES, "minHoleVertices"_a=DEFAULT_MINHOLEVERTICES,
        "minBboxArea"_a=DEFAULT_MINBBOX, "zThresh"_a=DEFAULT_ZTHRESH,
        "normThresh"_a=DEFAULT_NORMTHRESH, "allowedClass"_a=DEFAULT_ALLOWEDCLASS);


#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
