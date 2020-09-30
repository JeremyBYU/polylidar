
#include "polylidar_pybind/polylidar_pybind.hpp"
#include <iostream>

using namespace Polylidar;

// Convert Numpy Array to PolylidarMatrix
template <typename T>
Polylidar::Matrix<T> py_array_to_matrix(py::array_t<T, py::array::c_style | py::array::forcecast> array,
                                        bool copy_data = false)
{
    // return std::vector<std::array<T, dim>>();
    // std::cout << "Calling py_array_to_matrix" << std::endl;
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

    py::bind_vector<std::vector<std::size_t>>(
        m, "VectorULongInt", py::buffer_protocol(),
        "Contiguous buffer of Uint64. Use np.asarray() to get to get numpy array.");
    py::bind_vector<std::vector<uint8_t>>(m, "VectorUInt8", py::buffer_protocol(),
                                          "Contiguous buffer of Uint8. Use np.asarray() to get to get numpy array.");
    py::bind_vector<std::vector<double>>(m, "VectorDouble", py::buffer_protocol(),
                                         "Contiguous buffer of Float64. Use np.asarray() to get to get numpy array.");
    py::bind_vector<std::vector<int>>(m, "VectorInt", py::buffer_protocol(),
                                      "Contiguous buffer of Int32. Use np.asarray() to get to get numpy array.");

    py::class_<Polylidar::Matrix<double>>(
        m, "MatrixDouble", py::buffer_protocol(),
        "Matrix (Double) representation of numpy array. Use np.asarray() to get numpy array.")
        .def(py::init<>(&py_array_to_matrix<double>), "Creates a Matrix", "points"_a, "copy"_a = false)
        .def_buffer([](Polylidar::Matrix<double>& m) -> py::buffer_info {
            return py::buffer_info(m.ptr,                                   /* Pointer to buffer */
                                   sizeof(double),                          /* Size of one scalar */
                                   py::format_descriptor<double>::format(), /* Python struct-style format descriptor */
                                   2UL,                                     /* Number of dimensions */
                                   {m.rows, m.cols},                        /* Buffer dimensions */
                                   {sizeof(double) * m.cols,                /* Strides (in bytes) for each index */
                                    sizeof(double)});
        });

    py::class_<Polylidar::Matrix<uint8_t>>(
        m, "MatrixUInt8", py::buffer_protocol(),
        "Matrix (UInt8) representation of numpy array. Use np.asarray() to get numpy array.")
        .def(py::init<>(&py_array_to_matrix<uint8_t>), "Creates a Matrix", "points"_a, "copy"_a = false)
        .def_buffer([](Polylidar::Matrix<uint8_t>& m) -> py::buffer_info {
            return py::buffer_info(m.ptr,                                   /* Pointer to buffer */
                                   sizeof(uint8_t),                          /* Size of one scalar */
                                   py::format_descriptor<uint8_t>::format(), /* Python struct-style format descriptor */
                                   2UL,                                     /* Number of dimensions */
                                   {m.rows, m.cols},                        /* Buffer dimensions */
                                   {sizeof(uint8_t) * m.cols,                /* Strides (in bytes) for each index */
                                    sizeof(uint8_t)});
        });

    docstring::ClassMethodDocInject(
        m, "MatrixDouble", "__init__",
        {{"points", "Matrix-like numpy array"}, {"copy", "Make copy of data or use numpy memory buffer."}}, true);

    py::class_<Polylidar::Matrix<float>>(
        m, "MatrixFloat", py::buffer_protocol(),
        "Matrix (Float) representation of numpy array. Use np.asarray() to get numpy array.")
        .def(py::init<>(&py_array_to_matrix<float>), "Creates a Matrix", "points"_a, "copy"_a = false)
        .def_buffer([](Polylidar::Matrix<float>& m) -> py::buffer_info {
            return py::buffer_info(m.ptr,                                  /* Pointer to buffer */
                                   sizeof(float),                          /* Size of one scalar */
                                   py::format_descriptor<float>::format(), /* Python struct-style format descriptor */
                                   2UL,                                    /* Number of dimensions */
                                   {m.rows, m.cols},                       /* Buffer dimensions */
                                   {sizeof(float) * m.cols,                /* Strides (in bytes) for each index */
                                    sizeof(float)});
        });

    py::class_<Polylidar::Matrix<size_t>>(
        m, "MatrixULongInt", py::buffer_protocol(),
        "Matrix (size_t[Uint64]) representation of numpy array. Use np.asarray() to get numpy array.")
        .def(py::init<>(&py_array_to_matrix<size_t>), "Creates a Matrix", "points"_a, "copy"_a = false)
        .def_buffer([](Polylidar::Matrix<size_t>& m) -> py::buffer_info {
            return py::buffer_info(m.ptr,                                   /* Pointer to buffer */
                                   sizeof(size_t),                          /* Size of one scalar */
                                   py::format_descriptor<size_t>::format(), /* Python struct-style format descriptor */
                                   2UL,                                     /* Number of dimensions */
                                   {m.rows, m.cols},                        /* Buffer dimensions */
                                   {sizeof(size_t) * m.cols,                /* Strides (in bytes) for each index */
                                    sizeof(size_t)});
        });

    py::class_<Polylidar::Matrix<int>>(
        m, "MatrixInt", py::buffer_protocol(),
        "Matrix (Int32) representation of numpy array. Use np.asarray() to get numpy array.")
        .def(py::init<>(&py_array_to_matrix<int>), "Creates a Matrix", "points"_a, "copy"_a = false)
        .def_buffer([](Polylidar::Matrix<int>& m) -> py::buffer_info {
            return py::buffer_info(m.ptr,                                /* Pointer to buffer */
                                   sizeof(int),                          /* Size of one scalar */
                                   py::format_descriptor<int>::format(), /* Python struct-style format descriptor */
                                   2UL,                                  /* Number of dimensions */
                                   {m.rows, m.cols},                     /* Buffer dimensions */
                                   {sizeof(int) * m.cols,                /* Strides (in bytes) for each index */
                                    sizeof(int)});
        });

    py::class_<Polygon>(
        m, "Polygon",
        "Contains the linear rings of a polygon. The elements of a linear ring are point indices into a point cloud.")
        .def(py::init<>(), "Creates a Polygon")
        .def_readonly("shell", &Polygon::shell, py::return_value_policy::copy, "Linear ring of our exterior shell.")
        // .def_readonly("holes", &polylidar::Polygon::holes, py::return_value_policy::copy)
        .def_property("holes", &Polygon::getHoles, &Polygon::setHoles,
                      "Set of linear rings of interior holes in our polygon");

    py::class_<MeshHelper::HalfEdgeTriangulation>(
        m, "HalfEdgeTriangulation",
        "This class hold all the datastructures in our meshes (vertices, triangles, halfedges, etc.).")
        .def(py::init<Matrix<double>&>(), "Creates a HalfEdge Triangular Mesh", "in_vertices"_a)
        .def("__repr__", [](const MeshHelper::HalfEdgeTriangulation& a) { return "<HalfEdgeTriangulation>"; })
        .def_readonly("vertices", &MeshHelper::HalfEdgeTriangulation::vertices, "Vertices in the mesh, N X 2 or N X 3")
        .def_readonly("triangles", &MeshHelper::HalfEdgeTriangulation::triangles, "Triangles in the mesh, K X 3")
        .def_readonly("halfedges", &MeshHelper::HalfEdgeTriangulation::halfedges,
                      "Half-edge mapping in the mesh, K X 3. Every triangle has three oriented half-edges. Each "
                      "half-edge has unique id which is mapped to the 2D index of all triangles, e.g. the half-edges "
                      "of triangle k are [3*k, 3*k + 1, 3*k + 2]. Halfedges array provides the twin/opposite/shared "
                      "half-edge id. e.g., The twin half-edge of first edge for the triangle k is halfedges(k, 0)")
        .def_readonly("triangle_normals", &MeshHelper::HalfEdgeTriangulation::triangle_normals,
                      "Triangle normals in the mesh (normalized), K X 3")
        .def_readonly("vertex_classes", &MeshHelper::HalfEdgeTriangulation::vertex_classes, "Vertex classes in the mesh, N X 1")
        .def_readonly("counter_clock_wise", &MeshHelper::HalfEdgeTriangulation::counter_clock_wise,
                      "Direction of travel for oriented half-edges around a triangle")
        .def("set_triangle_normals", &MeshHelper::HalfEdgeTriangulation::SetTriangleNormals,
             "Sets Triangle Normals from input", "triangle_normals"_a)
        .def("set_vertex_classes", &MeshHelper::HalfEdgeTriangulation::SetVertexClasses,
             "Sets vertex classes from input", "vertex_classes"_a, "copy"_a=true)
        .def("compute_triangle_normals", &MeshHelper::HalfEdgeTriangulation::ComputeTriangleNormals,
             "Computes Triangle Normals");

    py::class_<Delaunator::Delaunator, MeshHelper::HalfEdgeTriangulation>(m, "Delaunator",
                                                                          "2D Delaunay Triangulation Class")
        .def(py::init<Matrix<double>>(), "Creates a 2D Delaunay triangulation of a point set", "in_vertices"_a)
        .def("__repr__", [](const Delaunator::Delaunator& dl) { return "<Delaunator>"; })
        .def("triangulate", &Delaunator::Delaunator::triangulate, "Triangulate the point set.");

    py::class_<Polylidar::Polylidar3D>(m, "Polylidar3D",
                                       "This class handles all data inputs: 2D points sets, 3D point clouds, 3D "
                                       "meshes. Will extract polygons from data.")
        .def(py::init<const double, const double, const size_t, const size_t, const double, const double, const double,
                      const int>(),
             "Constructs a Polylidar3D object.", "alpha"_a = PL_DEFAULT_ALPHA, "lmax"_a = PL_DEFAULT_LMAX,
             "min_triangles"_a = PL_DEFAULT_MINTRIANGLES, "min_hole_vertices"_a = PL_DEFAULT_MINHOLEVERTICES,
             "z_thresh"_a = PL_DEFAULT_ZTHRESH, "norm_thresh"_a = PL_DEFAULT_NORMTHRESH,
             "norm_thresh_min"_a = PL_DEFAULT_NORMTHRESH_MIN, "task_threads"_a = PL_DEFAULT_TASK_THREADS)
        .def("extract_planes_and_polygons",
             py::overload_cast<const Matrix<double>&, const std::array<double, 3>>(
                 &Polylidar::Polylidar3D::ExtractPlanesAndPolygons),
             "Extract Planes and Polygons from a 2D point sets or unorganized 3D point clouds. Uses 2D Delaunay "
             "triangulation for 2D point sets. Uses 2.5 Delaunay triangulation for 3D point clouds. Only send 3D point "
             "clouds whose desired surface for extraction is already aligned with the XY Plane (e.g., airborne LiDAR "
             "point clouds)",
             "points"_a, "plane_normal"_a = PL_DEFAULT_DESIRED_VECTOR)
        .def("extract_planes_and_polygons",
             py::overload_cast<MeshHelper::HalfEdgeTriangulation&, const std::array<double, 3>>(
                 &Polylidar::Polylidar3D::ExtractPlanesAndPolygons),
             "Extracts planes and polygons from a half-edge triangular mesh given a plane normal", "mesh"_a,
             "plane_normal"_a = PL_DEFAULT_DESIRED_VECTOR)
        .def("extract_planes_and_polygons_optimized",
             py::overload_cast<MeshHelper::HalfEdgeTriangulation&, const std::array<double, 3>>(
                 &Polylidar::Polylidar3D::ExtractPlanesAndPolygonsOptimized),
             "Extracts planes and polygons from a half-edge triangular mesh given a plane normal and makes use of "
             "task-based parallelism",
             "mesh"_a, "plane_normal"_a = PL_DEFAULT_DESIRED_VECTOR)
        .def("extract_planes_and_polygons",
             py::overload_cast<MeshHelper::HalfEdgeTriangulation&, const Matrix<double>&>(
                 &Polylidar::Polylidar3D::ExtractPlanesAndPolygons),
             "Extracts planes and polygons from a half-edge triangular mesh given **multiple** dominant plane normals.",
             "mesh"_a, "plane_normals"_a)
        .def("extract_planes_and_polygons_optimized",
             py::overload_cast<MeshHelper::HalfEdgeTriangulation&, const Matrix<double>&>(
                 &Polylidar::Polylidar3D::ExtractPlanesAndPolygonsOptimized),
             "Extracts planes and polygons from a half-edge triangular mesh given **multiple** dominant plane normals. "
             "Uses task-based parallelism.",
             "mesh"_a, "plane_normals"_a)
        .def("extract_planes_and_polygons_optimized_classified",
             &Polylidar::Polylidar3D::ExtractPlanesAndPolygonsOptimizedClassified,
             "Extracts planes and polygons from a classified half-edge triangular mesh given **multiple** dominant plane normals. "
             "Uses task-based parallelism.",
             "mesh"_a, "plane_normals"_a)
        .def("extract_tri_set", &Polylidar::Polylidar3D::ExtractTriSet,
             "Extract the triangle set (dominant planar grouping) for each triangle. This is used for "
             "debugging/visualization.",
             "mesh"_a, "plane_normals"_a)
        .def_readwrite(
            "alpha", &Polylidar3D::alpha,
            "Maximum circumcircle radius of a triangle. Filters 'big' triangles. Only applicable for 2D point sets.")
        .def_readwrite("lmax", &Polylidar3D::lmax,
                       "Maximum triangle edge length of a triangle in a planar segment. Filters 'big' triangles.")
        .def_readwrite(
            "min_triangles", &Polylidar3D::min_triangles,
            "Minimum number of triangles in a planar triangle segment. Filters small planar segments very fast.")
        .def_readwrite("min_hole_vertices", &Polylidar3D::min_hole_vertices,
                       "Minimum number of vertices for a hole in a polygon. Filters small holes very fast.")
        .def_readwrite("z_thresh", &Polylidar3D::z_thresh,
                       "Maximum point to plane distance during region growing (3D only). Forces planarity constraints. A value of 0.0 disables this constraint.")
        .def_readwrite("norm_thresh", &Polylidar3D::norm_thresh, "IGNORE - will be deprecated or repurposed (3D only)")
        .def_readwrite("norm_thresh_min", &Polylidar3D::norm_thresh_min,
                       "Minimum value of the dot product between a triangle and surface normal being extracted. Forces "
                       "planar constraints.")
        .def("__repr__", [](const Polylidar::Polylidar3D& pl) { return "<Polylidar::Polylidar3D>"; });

    // This doesn't work.... __init__ just isn't found I'm guessing
    docstring::ClassMethodDocInject(m, "Polylidar3D", "__init__",
                                    {{"alpha", "Maximum circumcircle radius of a triangle. Filters 'big' triangles. Only applicable for 2D point sets. A value of 0.0 makes this parameter ignored."},
                                     {"lmax", "Maximum triangle edge length of a triangle in a planar segment. Filters 'big' triangles."},
                                     {"min_triangles", "Minimum number of triangles in a planar triangle segment. Filters small planar segments very fast."},
                                     {"min_hole_vertices", "Minimum number of vertices for a hole in a polygon. Filters small holes very fast."},
                                     {"z_thresh", "Maximum point to plane distance during region growing (3D only). Forces planarity constraints. A value of 0.0 ignores this constraint."},
                                     {"norm_thresh", "IGNORE - will be deprecated or repurposed (3D only)"},
                                     {"norm_thresh_min", "Minimum value of the dot product between a triangle and surface normal being extracted. Forces planarity constraint."},
                                     {"task_threads", "Number of task threads that can be spawned."}}, true);

    docstring::ClassMethodDocInject(m, "Polylidar3D", "extract_tri_set", {{"mesh", ""}, {"plane_normals", ""}});

    m.def("extract_point_cloud_from_float_depth", &MeshHelper::ExtractPointCloudFromFloatDepth,
          "Extacts an organized point cloud from a depth image", "image"_a, "intrinsics"_a, "extrinsics"_a,
          "stride"_a = PL_DEFAULT_STRIDE);

    m.def("bilateral_filter_normals", &MeshHelper::BilateralFilterNormals,
          "Perform Bilateral Filtering on the triangular mesh. Only the normals are smoothed. Neighbors are in 1-ring "
          "vertex adjacency",
          "mesh"_a, "iterations"_a, "sigma_length"_a, "sigma_angle"_a);

    m.def("extract_tri_mesh_from_float_depth", &MeshHelper::ExtractTriMeshFromFloatDepth,
          "Extracts a Half-Edge Triangulated mesh (Uniform Mesh/Right Cut Mesh) from a depth image", "image"_a,
          "intrinsics"_a, "extrinsics"_a, "stride"_a = PL_DEFAULT_STRIDE, "calc_normals"_a = PL_DEFAULT_CALC_NORMALS);

    m.def("extract_tri_mesh_from_organized_point_cloud", &MeshHelper::ExtractTriMeshFromOrganizedPointCloud,
          "Extracts a Half-Edge Triangulated mesh (Uniform Mesh/Right Cut Mesh) from an organized point cloud. Returns "
          "mesh and T_map. T_map is the valid triangle set that maps between the complete fully connected mesh. and "
          "all triangles returned. You will mostly likely *not* need T_map and can safely discard.",
          "points"_a, "rows"_a, "cols"_a, "stride"_a = PL_DEFAULT_STRIDE, "calc_normals"_a = PL_DEFAULT_CALC_NORMALS);

    m.def("create_tri_mesh_copy",
          py::overload_cast<Matrix<double>&, Matrix<int>&, const bool>(&MeshHelper::CreateTriMeshCopy),
          "Creates a copy of a tri mesh, triangles of int dtype. Mostly used for Open3D triangles.", "vertices"_a,
          "triangles"_a, "calc_normals"_a = PL_DEFAULT_CALC_NORMALS);

    m.def("get_polylidar_version", &GetPolylidarVersion, "Get Polylidar Version");
    m.def("robust_predicates_activated", &RobustPredicatesActivated, "Check if built with Robust Geometric Predicates");

    docstring::FunctionDocInject(m, "create_tri_mesh_copy",
                                 {{"vertices", "Triangle vertices in Matrix Form"},
                                  {"triangles", "Triangle point indices in Matrix Form"},
                                  {"calc_normals", "Compute triangle normals."}});
}