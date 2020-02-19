// This glues pybind11 to the pure C++ polylidar project
// Jeremy Castagno

#include "polylidar/polylidar.hpp"
#include "delaunator.hpp"

#include "pybind11/pybind11.h" // Pybind11 import to define Python bindings
#include "pybind11/stl.h"      // Pybind11 import for STL containers
#include <pybind11/stl_bind.h> // Pybind11 stl bindings
#include "pybind11/numpy.h"

namespace py = pybind11;

namespace polylidar
{

std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>> _extractPlanesAndPolygons(py::array_t<double> nparray,
                                                                                                                     double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax = DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                                                                                                     size_t minHoleVertices = DEFAULT_MINHOLEVERTICES, double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                                                                                                     double normThresh = DEFAULT_NORMTHRESH, double normThreshMin = DEFAULT_NORMTHRESH_MIN,
                                                                                                                     double allowedClass = DEFAULT_ALLOWEDCLASS)
{
    // This function allows us to convert keyword arguments into a configuration struct
    auto info = nparray.request();
    std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
    Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minHoleVertices, minBboxArea, zThresh, normThresh, normThreshMin, allowedClass};
    Matrix<double> points((double *)info.ptr, shape[0], shape[1]);
    return ExtractPlanesAndPolygons(points, config);
}

std::tuple<std::vector<std::vector<size_t>>, std::vector<Polygon>> _extractPlanesAndPolygonsFromMesh(py::array_t<double> vertices, py::array_t<size_t> triangles, py::array_t<size_t> halfedges,
                                                                                                     double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax = DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                                                                                     size_t minHoleVertices = DEFAULT_MINHOLEVERTICES, double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                                                                                     double normThresh = DEFAULT_NORMTHRESH, double normThreshMin = DEFAULT_NORMTHRESH_MIN,
                                                                                                     double allowedClass = DEFAULT_ALLOWEDCLASS, std::array<double, 3> desiredVector = DEFAULT_DESIRED_VECTOR)
{
    // This function allows us to convert keyword arguments into a configuration struct
    auto info = vertices.request();
    std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
    Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minHoleVertices, minBboxArea, zThresh, normThresh, normThreshMin, allowedClass, desiredVector};
    Matrix<double> points((double *)info.ptr, shape[0], shape[1]);
    delaunator::HalfEdgeTriangulation triangulation(points, triangles, halfedges);
    return ExtractPlanesAndPolygonsFromMesh(triangulation, config);
}

std::vector<double> _extractPointCloudFromFloatDepth(py::array_t<float> image, py::array_t<double> intrinsics, py::array_t<double> extrinsics, size_t stride = DEFAULT_STRIDE)
{
    // Create Image Wrapper
    auto info_im = image.request();
    Matrix<float> im((float *)info_im.ptr, info_im.shape[0], info_im.shape[1]);
    // Create intrinsics wrapper
    auto info_int = intrinsics.request();
    Matrix<double> intrinsics_((double *)info_int.ptr, info_int.shape[0], info_int.shape[1]);
    // Create extrinsics wrapper
    auto info_ext = extrinsics.request();
    Matrix<double> extrinsics_((double *)info_ext.ptr, info_ext.shape[0], info_ext.shape[1]);
    // Extract point cloud
    std::vector<double> points = ExtractPointCloudFromFloatDepth(im, intrinsics_, extrinsics_, stride);
    // std::cout << "extractPointCloudFromFloatDepth C++ : " << points[0] << " Address:" <<  &points[0] << std::endl;

    return points;
}

delaunator::TriMesh _extractTriMeshFromFloatDepth(py::array_t<float> image, py::array_t<double> intrinsics, py::array_t<double> extrinsics, size_t stride = DEFAULT_STRIDE, const bool calc_normals = DEFAULT_CALC_NORMALS)
{
    // Will hold the point cloud
    std::vector<double> points;
    std::vector<size_t> triangles;
    std::vector<size_t> halfedges;
    // Create Image Wrapper
    auto info_im = image.request();
    Matrix<float> im((float *)info_im.ptr, info_im.shape[0], info_im.shape[1]);
    // Create intrinsics wrapper
    auto info_int = intrinsics.request();
    Matrix<double> intrinsics_((double *)info_int.ptr, info_int.shape[0], info_int.shape[1]);
    // Create extrinsics wrapper
    auto info_ext = extrinsics.request();
    Matrix<double> extrinsics_((double *)info_ext.ptr, info_ext.shape[0], info_ext.shape[1]);

    // Get Data
    auto triMesh = ExtractTriMeshFromFloatDepth(im, intrinsics_, extrinsics_, stride, calc_normals);
    // std::cout << "_extractUniformMeshFromFloatDepth C++ : " << points[0] << " Address:" <<  &points[0] << std::endl;

    return triMesh;
}

std::vector<Polygon> _extractPolygonsFromMesh(py::array_t<double> vertices, py::array_t<size_t> triangles, py::array_t<size_t> halfedges,
                                              double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax = DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                              size_t minHoleVertices = DEFAULT_MINHOLEVERTICES, double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                              double normThresh = DEFAULT_NORMTHRESH, double normThreshMin = DEFAULT_NORMTHRESH_MIN,
                                              double allowedClass = DEFAULT_ALLOWEDCLASS)
{
    // This function allows us to convert keyword arguments into a configuration struct
    auto info = vertices.request();
    std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
    Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minHoleVertices, minBboxArea, zThresh, normThresh, normThreshMin, allowedClass};
    Matrix<double> points((double *)info.ptr, shape[0], shape[1]);
    delaunator::HalfEdgeTriangulation triangulation(points, triangles, halfedges);
    return ExtractPolygonsFromMesh(triangulation, config);
}

std::vector<Polygon> _extractPolygons(py::array_t<double> nparray,
                                      double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax = DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                      size_t minHoleVertices = DEFAULT_MINHOLEVERTICES, double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                      double normThresh = DEFAULT_NORMTHRESH, double normThreshMin = DEFAULT_NORMTHRESH_MIN,
                                      double allowedClass = DEFAULT_ALLOWEDCLASS)
{
    // This function allows us to convert keyword arguments into a configuration struct
    auto info = nparray.request();
    std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
    Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minHoleVertices, minBboxArea, zThresh, normThresh, normThreshMin, allowedClass};
    Matrix<double> points((double *)info.ptr, shape[0], shape[1]);
    return ExtractPolygons(points, config);
}

std::tuple<std::vector<Polygon>, std::vector<float>> _extractPolygonsAndTimings(py::array_t<double> nparray,
                                                                                double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax = DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                                                                size_t minHoleVertices = DEFAULT_MINHOLEVERTICES, double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                                                                double normThresh = DEFAULT_NORMTHRESH, double allowedClass = DEFAULT_ALLOWEDCLASS)
{
    auto info = nparray.request();
    std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
    // This function allows us to convert keyword arguments into a configuration struct
    Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minHoleVertices, minBboxArea, zThresh, normThresh, 0.5, allowedClass};
    Matrix<double> points((double *)info.ptr, shape[0], shape[1]);
    std::vector<float> timings;
    auto polygons = ExtractPolygonsAndTimings(points, config, timings);

    return std::make_tuple(polygons, timings);
}

} // namespace polylidar