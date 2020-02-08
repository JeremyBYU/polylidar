// This glues pybind11 to the pure C++ polylidar project
// Jeremy Castagno

#include "polylidar/polylidar.hpp"
#include "delaunator.hpp"

#include "pybind11/pybind11.h" // Pybind11 import to define Python bindings
#include "pybind11/stl.h"      // Pybind11 import for STL containers
#include <pybind11/stl_bind.h> // Pybind11 stl bindings
#include "pybind11/numpy.h"


namespace py = pybind11;


namespace polylidar {

  std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>> extractPlanesAndPolygons(py::array_t<double> nparray,
                                                                                                                    double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax=DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                                                                                                    size_t minHoleVertices = DEFAULT_MINHOLEVERTICES, double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                                                                                                    double normThresh = DEFAULT_NORMTHRESH, double normThreshMin = DEFAULT_NORMTHRESH_MIN, 
                                                                                                                    double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        // This function allows us to convert keyword arguments into a configuration struct
        auto info = nparray.request();
        std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
        Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minHoleVertices, minBboxArea, zThresh, normThresh, normThreshMin,  allowedClass};
        Matrix<double> points((double*)info.ptr, shape[0], shape[1]);
        return _extractPlanesAndPolygons(points, config);
    }

    std::tuple<std::vector<std::vector<size_t>>, std::vector<Polygon>> _extractPlanesAndPolygonsFromMesh(py::array_t<double> vertices, py::array_t<size_t> triangles, py::array_t<size_t> halfedges,
                                                                                                                    double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax=DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                                                                                                    size_t minHoleVertices = DEFAULT_MINHOLEVERTICES, double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                                                                                                    double normThresh = DEFAULT_NORMTHRESH, double normThreshMin = DEFAULT_NORMTHRESH_MIN, 
                                                                                                                    double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        // This function allows us to convert keyword arguments into a configuration struct
        auto info = vertices.request();
        std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
        Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minHoleVertices, minBboxArea, zThresh, normThresh, normThreshMin,  allowedClass};
        Matrix<double> points((double*)info.ptr, shape[0], shape[1]);
        delaunator::HalfEdgeTriangulation triangulation(points, triangles, halfedges);
        return extractPlanesAndPolygonsFromMesh(triangulation, config);
    }

    std::vector<double> _extractPointCloudFromFloatDepth(py::array_t<float> image, py::array_t<double> intrinsics, size_t stride=DEFAULT_STRIDE)
    {          
        // Create Image Wrapper
        auto info_im = image.request();
        Matrix<float> im((float*)info_im.ptr, info_im.shape[0], info_im.shape[1]);
        // Create intrinsics wrapper
        auto info_int = intrinsics.request();
        Matrix<double> intrinsics_((double*)info_int.ptr, info_int.shape[0], info_int.shape[1]);
        // Extract point cloud
        std::vector<double> points = extractPointCloudFromFloatDepth(im, intrinsics_, stride);
        // std::cout << "extractPointCloudFromFloatDepth C++ : " << points[0] << " Address:" <<  &points[0] << std::endl;

        return points;
    }

    std::tuple<std::vector<double>, std::vector<size_t>, std::vector<size_t>> _extractUniformMeshFromFloatDepth(py::array_t<float> image, py::array_t<double> intrinsics, size_t stride=DEFAULT_STRIDE)
    {   
        // Will hold the point cloud
        std::vector<double> points;
        std::vector<size_t> triangles;
        std::vector<size_t> halfedges;
        // Create Image Wrapper
        auto info_im = image.request();
        Matrix<float> im((float*)info_im.ptr, info_im.shape[0], info_im.shape[1]);
        // Create intrinsics wrapper
        auto info_int = intrinsics.request();
        Matrix<double> intrinsics_((double*)info_int.ptr, info_int.shape[0], info_int.shape[1]);

        // Get Data
        std::tie(points, triangles, halfedges) = extractUniformMeshFromFloatDepth(im, intrinsics_, stride);
        std::cout << "_extractUniformMeshFromFloatDepth C++ : " << points[0] << " Address:" <<  &points[0] << std::endl;
        

        return std::make_tuple(std::move(points), std::move(triangles), std::move(halfedges));

    }


        // m.def("extract_point_cloud_from_float_depth", &polylidar::_extractPointCloudFromFloatDepth, "Extracts point cloud from a float depth image",
        // "image"_a, "intrinsics"_a, "stride"_a=DEFAULT_STRIDE);

    std::vector<Polygon> _extractPolygonsFromMesh(py::array_t<double> vertices, py::array_t<size_t> triangles, py::array_t<size_t> halfedges,
                                                double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax=DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                                size_t minHoleVertices = DEFAULT_MINHOLEVERTICES, double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                                double normThresh = DEFAULT_NORMTHRESH, double normThreshMin = DEFAULT_NORMTHRESH_MIN, 
                                                double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        // This function allows us to convert keyword arguments into a configuration struct
        auto info = vertices.request();
        std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
        Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minHoleVertices, minBboxArea, zThresh, normThresh, normThreshMin,  allowedClass};
        Matrix<double> points((double*)info.ptr, shape[0], shape[1]);
        delaunator::HalfEdgeTriangulation triangulation(points, triangles, halfedges);
        return extractPolygonsFromMesh(triangulation, config);
    }


    std::vector<Polygon> extractPolygons(py::array_t<double> nparray,
                                        double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax=DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                        size_t minHoleVertices = DEFAULT_MINHOLEVERTICES, double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                        double normThresh = DEFAULT_NORMTHRESH, double normThreshMin = DEFAULT_NORMTHRESH_MIN,
                                        double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        // This function allows us to convert keyword arguments into a configuration struct
        auto info = nparray.request();
        std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
        Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minHoleVertices, minBboxArea, zThresh, normThresh, normThreshMin, allowedClass};
        Matrix<double> points((double*)info.ptr, shape[0], shape[1]);
        return _extractPolygons(points, config);
    }

    std::tuple<std::vector<Polygon>, std::vector<float>> extractPolygonsAndTimings(py::array_t<double> nparray,
                                        double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax=DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                        size_t minHoleVertices = DEFAULT_MINHOLEVERTICES, double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                        double normThresh = DEFAULT_NORMTHRESH, double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        auto info = nparray.request();
        std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
        // This function allows us to convert keyword arguments into a configuration struct
        Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minHoleVertices, minBboxArea, zThresh, normThresh, 0.5, allowedClass};
        Matrix<double> points((double*)info.ptr, shape[0], shape[1]);
        std::vector<float> timings;
        auto polygons = _extractPolygonsAndTimings(points, config, timings);
        
        return std::make_tuple(polygons, timings);
    }



}