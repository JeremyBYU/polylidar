
#include "polylidar_pybind/polylidar_pybind.hpp"

using namespace Polylidar;

// Convert Numpy Array to PolylidarMatrix
template <typename T>
Polylidar::Matrix<T> py_array_to_matrix(py::array_t<T, py::array::c_style | py::array::forcecast> array,
                                        bool copy_data = false)
{
    // return std::vector<std::array<T, dim>>();
    if (array.ndim() != 2)
    {
        throw py::cast_error("Numpy array must have exactly 2 Dimensions to be transformed to Polylidar::Matrix<T>");
    }
    size_t rows = array.shape(0);
    size_t cols = array.shape(1);

    if (copy_data)
    {
        size_t total_elements = rows * cols;
        std::vector<T> vectors_T(total_elements);
        // auto array_unchecked = array.mutable_unchecked<2UL>();
        size_t element_counter = 0;
        for (size_t i = 0; i < rows; ++i)
        {
            for (size_t j = 0; j < cols; ++j)
            {
                vectors_T[element_counter] = array.at(i, j);
                element_counter++;
            }
        }
        Polylidar::Matrix<T> new_matrix(std::move(vectors_T), rows, cols);
        return new_matrix;
    }
    else
    {
        auto info = array.request();
        Polylidar::Matrix<T> new_matrix(static_cast<T*>(info.ptr), rows, cols);
        return new_matrix;
    }
}

PYBIND11_MODULE(polylidar, m)
{
    m.doc() = "Python binding of Polylidar";

    py::bind_vector<std::vector<std::size_t>>(m, "VectorULongInt", py::buffer_protocol());
    py::bind_vector<std::vector<uint8_t>>(m, "VectorUInt8", py::buffer_protocol());
    py::bind_vector<std::vector<double>>(m, "VectorDouble", py::buffer_protocol());
    py::bind_vector<std::vector<int>>(m, "VectorInt", py::buffer_protocol());

    py::class_<Polylidar::Matrix<double>>(m, "MatrixDouble", py::buffer_protocol())
        .def(py::init<>(&py_array_to_matrix<double>), "points"_a, "copy"_a = false)
        .def_buffer([](Polylidar::Matrix<double>& m) -> py::buffer_info {
            return py::buffer_info(m.ptr,                                   /* Pointer to buffer */
                                   sizeof(double),                          /* Size of one scalar */
                                   py::format_descriptor<double>::format(), /* Python struct-style format descriptor */
                                   2UL,                                     /* Number of dimensions */
                                   {m.rows, m.cols},                        /* Buffer dimensions */
                                   {sizeof(double) * m.cols,                /* Strides (in bytes) for each index */
                                    sizeof(double)});
        });

    py::class_<Polylidar::Matrix<size_t>>(m, "MatrixULongInt", py::buffer_protocol())
        .def(py::init<>(&py_array_to_matrix<size_t>), "points"_a, "copy"_a = false)
        .def_buffer([](Polylidar::Matrix<size_t>& m) -> py::buffer_info {
            return py::buffer_info(m.ptr,                                   /* Pointer to buffer */
                                   sizeof(size_t),                          /* Size of one scalar */
                                   py::format_descriptor<size_t>::format(), /* Python struct-style format descriptor */
                                   2UL,                                     /* Number of dimensions */
                                   {m.rows, m.cols},                        /* Buffer dimensions */
                                   {sizeof(size_t) * m.cols,                /* Strides (in bytes) for each index */
                                    sizeof(size_t)});
        });

    py::class_<MeshHelper::HalfEdgeTriangulation>(m, "HalfEdgeTriangulation")
        .def(py::init<Matrix<double>&>(), "in_vertices"_a)
        .def("__repr__", [](const MeshHelper::HalfEdgeTriangulation& a) { return "<HalfEdgeTriangulation>"; })
        .def_readonly("vertices", &MeshHelper::HalfEdgeTriangulation::vertices)
        .def_readonly("triangles", &MeshHelper::HalfEdgeTriangulation::triangles)
        .def_readonly("halfedges", &MeshHelper::HalfEdgeTriangulation::halfedges);

    py::class_<Delaunator::Delaunator, MeshHelper::HalfEdgeTriangulation>(m, "Delaunator")
        .def(py::init<Matrix<double>>(), "in_vertices"_a)
        .def("__repr__", [](const Delaunator::Delaunator& dl) { return "<Delaunator>"; })
        .def("triangulate", &Delaunator::Delaunator::triangulate);

    py::class_<Polylidar::Polylidar3D>(m, "Polylidar3D")
        .def(py::init<const double, const double, const size_t, const size_t, const double, const double,
                      const double>(),
             "alpha"_a = PL_DEFAULT_ALPHA, "lmax"_a = PL_DEFAULT_LMAX, "min_triangles"_a = PL_DEFAULT_MINTRIANGLES,
             "min_hole_vertices"_a = PL_DEFAULT_MINHOLEVERTICES, "z_thresh"_a = PL_DEFAULT_ZTHRESH,
             "norm_thresh"_a = PL_DEFAULT_NORMTHRESH, "norm_thresh_min"_a = PL_DEFAULT_NORMTHRESH_MIN)
        .def("__repr__", [](const Polylidar::Polylidar3D& pl) { return "<Polylidar::Polylidar3D>"; });

    // Functions
    // m.def("convert_normals_to_hilbert", &FastGA::Helper::ConvertNormalsToHilbert, "normals"_a, "bbox"_a);
    // m.def("convert_normals_to_s2id", &FastGA::Helper::ConvertNormalsToS2ID, "normals"_a);
    // m.def("refine_icosahedron", &FastGA::Ico::RefineIcosahedron, "level"_a);
    // m.def("refine_icochart", &FastGA::Ico::RefineIcoChart, "level"_a = 0, "square"_a = false);
}