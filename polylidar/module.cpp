#include "polylidar.hpp"

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::vector<size_t>);
PYBIND11_MAKE_OPAQUE(std::vector<double>);

using namespace pybind11::literals;

// double DESIRED_VECTOR_2[3] = {0.0, 0.0, 1.0};

PYBIND11_MODULE(polylidar, m)
{

    // xt::import_numpy(); // MUST import numpy here
    m.doc() = R"pbdoc(
        Pybind11 of polylidar
        -----------------------

        .. currentmodule:: polylidar

        .. autosummary::
           :toctree: _generate

    )pbdoc";

    py::bind_vector<std::vector<std::size_t>>(m, "VectorInts", py::buffer_protocol());
    py::bind_vector<std::vector<double>>(m, "VectorDouble", py::buffer_protocol());

    py::class_<delaunator::Delaunator>(m, "Delaunator")
        .def(py::init<py::array_t<double>>())
        .def("triangulate", &delaunator::Delaunator::triangulate)
        .def_readonly("triangles", &delaunator::Delaunator::triangles)
        .def_readonly("halfedges", &delaunator::Delaunator::halfedges)
        .def_readonly("coords", &delaunator::Delaunator::coords);

    py::class_<polylidar::Polygon>(m, "Polygon")
        .def(py::init<>())
        .def_readonly("shell", &polylidar::Polygon::shell, py::return_value_policy::copy)
        // .def_readonly("holes", &polylidar::Polygon::holes, py::return_value_policy::copy)
        .def_property("holes", &polylidar::Polygon::getHoles, &polylidar::Polygon::setHoles);
    
    m.def("extractPlanesAndPolygons", &polylidar::extractPlanesAndPolygons,
        "nparray"_a, "dim"_a=DEFAULT_DIM, "alpha"_a=DEFAULT_ALPHA, "xyThresh"_a=DEFAULT_XYTHRESH,
        "minTriangles"_a=DEFAULT_MINTRIANGLES,
        "minBboxArea"_a=DEFAULT_MINBBOX, "zThresh"_a=DEFAULT_ZTHRESH,
        "normThresh"_a=DEFAULT_NORMTHRESH, "allowedClass"_a=DEFAULT_ALLOWEDCLASS);

    m.def("extractPolygons", &polylidar::extractPolygons,
        "nparray"_a, "dim"_a=DEFAULT_DIM, "alpha"_a=DEFAULT_ALPHA, "xyThresh"_a=DEFAULT_XYTHRESH,
        "minTriangles"_a=DEFAULT_MINTRIANGLES,
        "minBboxArea"_a=DEFAULT_MINBBOX, "zThresh"_a=DEFAULT_ZTHRESH,
        "normThresh"_a=DEFAULT_NORMTHRESH, "allowedClass"_a=DEFAULT_ALLOWEDCLASS);


#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
