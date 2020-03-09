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

    py::class_<polylidar::Matrix<double>>(m, "Matrix<double>", py::buffer_protocol())
        .def_buffer([](polylidar::Matrix<double> &m) -> py::buffer_info {
            return py::buffer_info(
                m.ptr,                                   /* Pointer to buffer */
                sizeof(double),                          /* Size of one scalar */
                py::format_descriptor<double>::format(), /* Python struct-style format descriptor */
                2,                                       /* Number of dimensions */
                {m.rows, m.cols},                        /* Buffer dimensions */
                {sizeof(double) * m.cols,                /* Strides (in bytes) for each index */
                 sizeof(double)});
        });

    py::class_<delaunator::Delaunator>(m, "Delaunator")
        .def(py::init<py::array_t<double>>())
        .def("triangulate", &delaunator::Delaunator::triangulate)
        .def_readonly("triangles", &delaunator::Delaunator::triangles)
        .def_readonly("halfedges", &delaunator::Delaunator::halfedges)
        .def_readonly("hull_tri", &delaunator::Delaunator::hull_tri)
        .def_readonly("coords", &delaunator::Delaunator::coords);

    py::class_<polylidar::MeshHelper::HalfEdgeTriangulation>(m, "HalfEdgeTriangulation")
        // .def(py::init<py::array_t<double>>())
        .def_readonly("triangles", &polylidar::MeshHelper::HalfEdgeTriangulation::triangles)
        .def_readonly("halfedges", &polylidar::MeshHelper::HalfEdgeTriangulation::halfedges)
        .def_readonly("coords", &polylidar::MeshHelper::HalfEdgeTriangulation::coords);

    py::class_<polylidar::MeshHelper::TriMesh>(m, "TriMesh")
        // .def(py::init<std::vector<double>, std::vector<double>, std::vector<size_t>>())
        .def_readonly("vertices", &polylidar::MeshHelper::TriMesh::vertices)
        .def_readonly("triangles", &polylidar::MeshHelper::TriMesh::triangles)
        .def_readonly("halfedges", &polylidar::MeshHelper::TriMesh::halfedges)
        .def_readonly("triangle_normals", &polylidar::MeshHelper::TriMesh::triangle_normals)
        .def_readonly("coords", &polylidar::MeshHelper::TriMesh::coords);

    py::class_<polylidar::Polygon>(m, "Polygon")
        .def(py::init<>())
        .def_readonly("shell", &polylidar::Polygon::shell, py::return_value_policy::copy)
        // .def_readonly("holes", &polylidar::Polygon::holes, py::return_value_policy::copy)
        .def_property("holes", &polylidar::Polygon::getHoles, &polylidar::Polygon::setHoles);

    m.def("extractPlanesAndPolygons", &polylidar::_extractPlanesAndPolygons, "Extracts planar meshes and polygons from a point cloud",
          "nparray"_a, "alpha"_a = DEFAULT_ALPHA, "xyThresh"_a = DEFAULT_XYTHRESH,
          "lmax"_a = DEFAULT_LMAX, "minTriangles"_a = DEFAULT_MINTRIANGLES, "minHoleVertices"_a = DEFAULT_MINHOLEVERTICES,
          "minBboxArea"_a = DEFAULT_MINBBOX, "zThresh"_a = DEFAULT_ZTHRESH,
          "normThresh"_a = DEFAULT_NORMTHRESH, "normThreshMin"_a = DEFAULT_NORMTHRESH_MIN,
          "allowedClass"_a = DEFAULT_ALLOWEDCLASS);

    m.def("extractPolygons", &polylidar::_extractPolygons, "Extracts polygons from a point cloud",
          "nparray"_a, "alpha"_a = DEFAULT_ALPHA, "xyThresh"_a = DEFAULT_XYTHRESH,
          "lmax"_a = DEFAULT_LMAX, "minTriangles"_a = DEFAULT_MINTRIANGLES, "minHoleVertices"_a = DEFAULT_MINHOLEVERTICES,
          "minBboxArea"_a = DEFAULT_MINBBOX, "zThresh"_a = DEFAULT_ZTHRESH,
          "normThresh"_a = DEFAULT_NORMTHRESH, "normThreshMin"_a = DEFAULT_NORMTHRESH_MIN,
          "allowedClass"_a = DEFAULT_ALLOWEDCLASS);

    m.def("extractPolygonsAndTimings", &polylidar::_extractPolygonsAndTimings, "Extracts polygons from a point cloud and returns detailed timings of triangulation, mesh extraction, and polygon extraction",
          "nparray"_a, "alpha"_a = DEFAULT_ALPHA, "xyThresh"_a = DEFAULT_XYTHRESH,
          "lmax"_a = DEFAULT_LMAX, "minTriangles"_a = DEFAULT_MINTRIANGLES, "minHoleVertices"_a = DEFAULT_MINHOLEVERTICES,
          "minBboxArea"_a = DEFAULT_MINBBOX, "zThresh"_a = DEFAULT_ZTHRESH,
          "normThresh"_a = DEFAULT_NORMTHRESH, "allowedClass"_a = DEFAULT_ALLOWEDCLASS);

    m.def("extract_planes_and_polygons_from_mesh", &polylidar::_extractPlanesAndPolygonsFromMesh, "Extracts planar meshes and polygons from a half edge triangulated mesh",
          "triangulation"_a, "alpha"_a = DEFAULT_ALPHA, "xyThresh"_a = DEFAULT_XYTHRESH,
          "lmax"_a = DEFAULT_LMAX, "minTriangles"_a = DEFAULT_MINTRIANGLES, "minHoleVertices"_a = DEFAULT_MINHOLEVERTICES,
          "minBboxArea"_a = DEFAULT_MINBBOX, "zThresh"_a = DEFAULT_ZTHRESH,
          "normThresh"_a = DEFAULT_NORMTHRESH, "normThreshMin"_a = DEFAULT_NORMTHRESH_MIN,
          "allowedClass"_a = DEFAULT_ALLOWEDCLASS, "desiredVector"_a = DEFAULT_DESIRED_VECTOR);

    m.def("extract_polygons_from_mesh", &polylidar::_extractPolygonsFromMesh, "Extracts polygons from a half edge triangulated mesh",
          "triangulation"_a, "alpha"_a = DEFAULT_ALPHA, "xyThresh"_a = DEFAULT_XYTHRESH,
          "lmax"_a = DEFAULT_LMAX, "minTriangles"_a = DEFAULT_MINTRIANGLES, "minHoleVertices"_a = DEFAULT_MINHOLEVERTICES,
          "minBboxArea"_a = DEFAULT_MINBBOX, "zThresh"_a = DEFAULT_ZTHRESH,
          "normThresh"_a = DEFAULT_NORMTHRESH, "normThreshMin"_a = DEFAULT_NORMTHRESH_MIN,
          "allowedClass"_a = DEFAULT_ALLOWEDCLASS, "desiredVector"_a = DEFAULT_DESIRED_VECTOR);

    m.def("extract_point_cloud_from_float_depth", &polylidar::_extractPointCloudFromFloatDepth, "Extracts point cloud from a float depth image",
          "image"_a, "intrinsics"_a, "extrinsics"_a, "stride"_a = DEFAULT_STRIDE);

    m.def("extract_tri_mesh_from_float_depth", &polylidar::_extractTriMeshFromFloatDepth, "Extracts a uniform triangular mesh from a float depth image",
          "image"_a, "intrinsics"_a, "extrinsics"_a, "stride"_a = DEFAULT_STRIDE, "calc_normals"_a = DEFAULT_CALC_NORMALS);

    m.def("create_tri_mesh_copy", &polylidar::CreateTriMeshCopy, "Creates a copy of a triangular mesh",
          "vertices"_a, "triangles"_a);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
