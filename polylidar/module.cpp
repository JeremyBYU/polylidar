#include "polylidar.hpp"

namespace py = pybind11;

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

    py::class_<PyMultiAStar>(m, "PyMultiAStar")
        .def(py::init<pybind11::array_t<float>, bool, float, float, float, float, float, float, bool>(),
             py::arg("map"), py::arg("allow_diag") = true, py::arg("map_res") = RES, py::arg("obstacle_value") = LARGE_NUMBER, py::arg("path_w0") = W0,
             py::arg("normalizing_path_cost") = NORM_PATH_COST, py::arg("goal_weight") = GOAL_WEIGHT, py::arg("path_weight") = PATH_WEIGHT, py::arg("keep_nodes") = KEEP_NODES)
        .def("search_multiple", &PyMultiAStar::search_multiple, py::arg("start_cell"), py::arg("goal_cells"))
        .def("search_single", &PyMultiAStar::search_single_public, py::arg("start_cell"), py::arg("goal_cell"));

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
