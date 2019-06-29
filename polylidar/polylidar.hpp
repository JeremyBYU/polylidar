
#ifndef POLYLIDAR
#define POLYLIDAR
#define _USE_MATH_DEFINES
#define VERSION_INFO "0.0.3"

#include <array>
#include <ostream>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>

#include "helper.hpp"
#include "delaunator.hpp"
#include "robin_hood.h"

#include "pybind11/pybind11.h" // Pybind11 import to define Python bindings
#include "pybind11/stl.h"      // Pybind11 import for STL containers
#include <pybind11/stl_bind.h> // Pybind11 stl bindings
#include "pybind11/numpy.h"

#define DEFAULT_DIM 2
#define DEFAULT_ALPHA 1.0
#define DEFAULT_XYTHRESH 0.0
#define DEFAULT_LMAX 0.0
#define DEFAULT_MINTRIANGLES 20
#define DEFAULT_MINBBOX 100.0
#define DEFAULT_ZTHRESH 0.20
#define DEFAULT_NORMTHRESH 0.90
#define DEFAULT_ALLOWEDCLASS 4.0

#define DEBUG 1

namespace py = pybind11;

namespace polylidar {

    using vvi = std::vector<std::vector<size_t>>;
    struct Config
    {
        // 2D base configuration
        int dim = DEFAULT_DIM;
        double alpha = DEFAULT_ALPHA;
        double xyThresh = DEFAULT_XYTHRESH;
        double lmax = DEFAULT_LMAX;
        size_t minTriangles = DEFAULT_MINTRIANGLES;
        double minBboxArea = DEFAULT_MINBBOX;
        // 3D configuration
        double zThresh = DEFAULT_ZTHRESH;
        double normThresh = DEFAULT_NORMTHRESH;
        // 4D configuration
        double allowedClass = DEFAULT_ALLOWEDCLASS;
        std::array<double, 3> desiredVector = std::array<double, 3>{0.0, 0.0, 1.0};
    };

    struct Polygon {
        std::vector<size_t> shell;
        std::vector<std::vector<size_t>> holes;
        // I know this looks crazy
        // but for some reason I need this to allow the polygon to have holes
        // without it I could access the holes, but only ONCE. Then they would be gone
        // i.e in python "polygon.holes" <- Now its dead the next time you acces it
        // So I think this now makes a copy on every access. but whatever
        vvi getHoles() const {return holes;}
        void setHoles(vvi x) {holes = x;}
    };

    std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>>  _extractPlanesAndPolygons(pybind11::array_t<double> nparray, Config config);

    std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>>  extractPlanesAndPolygons(pybind11::array_t<double> nparray,
                                double alpha, double xyThresh, double lmax, size_t minTriangles,
                                double minBboxArea, double zThresh, 
                                double normThresh, double allowedClass);


    std::vector<Polygon> _extractPolygons(py::array_t<double> nparray, Config config);

    std::vector<Polygon>  extractPolygons(pybind11::array_t<double> nparray,
                                double alpha, double xyThresh, double lamx, size_t minTriangles,
                                double minBboxArea, double zThresh, 
                                double normThresh, double allowedClass);
    std::tuple<std::vector<Polygon>, std::vector<float>>  extractPolygonsAndTimings(pybind11::array_t<double> nparray,
                                double alpha, double xyThresh, double lmax, size_t minTriangles,
                                double minBboxArea, double zThresh, 
                                double normThresh, double allowedClass);
}


#endif
