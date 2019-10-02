#include "polylidar.hpp"
#include "delaunator.hpp"

#include "pybind11/pybind11.h" // Pybind11 import to define Python bindings
#include "pybind11/stl.h"      // Pybind11 import for STL containers
#include <pybind11/stl_bind.h> // Pybind11 stl bindings
#include "pybind11/numpy.h"


namespace py = pybind11;


namespace polylidar {

  std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>> extractPlanesAndPolygons(py::array_t<double> nparray,
                                                                                                                    double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax=DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                                                                                                    double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                                                                                                    double normThresh = DEFAULT_NORMTHRESH, double normThreshMin = DEFAULT_NORMTHRESH_MIN, 
                                                                                                                    double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        // This function allows us to convert keyword arguments into a configuration struct
        auto info = nparray.request();
        std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
        Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minBboxArea, zThresh, normThresh, normThreshMin,  allowedClass};
        Matrix points((double*)info.ptr, shape[0], shape[1]);
        return _extractPlanesAndPolygons(points, config);
    }


    std::vector<Polygon> extractPolygons(py::array_t<double> nparray,
                                        double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax=DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                        double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                        double normThresh = DEFAULT_NORMTHRESH, double normThreshMin = DEFAULT_NORMTHRESH_MIN,
                                        double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        // This function allows us to convert keyword arguments into a configuration struct
        auto info = nparray.request();
        std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
        Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minBboxArea, zThresh, normThresh, normThreshMin, allowedClass};
        Matrix points((double*)info.ptr, shape[0], shape[1]);
        return _extractPolygons(points, config);
    }

    std::tuple<std::vector<Polygon>, std::vector<float>> extractPolygonsAndTimings(py::array_t<double> nparray,
                                        double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, double lmax=DEFAULT_LMAX, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                        double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH,
                                        double normThresh = DEFAULT_NORMTHRESH, double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        auto info = nparray.request();
        std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
        // This function allows us to convert keyword arguments into a configuration struct
        Config config{shape[1], alpha, xyThresh, lmax, minTriangles, minBboxArea, zThresh, normThresh, 0.5, allowedClass};
        Matrix points((double*)info.ptr, shape[0], shape[1]);
        std::vector<float> timings;
        auto polygons = _extractPolygonsAndTimings(points, config, timings);
        
        return std::make_tuple(polygons, timings);
    }



}