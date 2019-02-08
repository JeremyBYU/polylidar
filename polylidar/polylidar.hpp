
#ifndef POLYLIDAR
#define POLYLIDAR

#include <array>
#include <ostream>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <chrono>

#include "helper.hpp"
#include "delaunator.hpp"

#include "pybind11/pybind11.h" // Pybind11 import to define Python bindings
#include "pybind11/stl.h"      // Pybind11 import for STL containers
#include <pybind11/stl_bind.h> // Pybind11 stl bindings
#include "pybind11/numpy.h"

#define DEFAULT_DIM 2
#define DEFAULT_ALPHA 1.0
#define DEFAULT_XYTHRESH 0.0
#define DEFAULT_MINTRIANGLES 20
#define DEFAULT_MINBBOX 100.0
#define DEFAULT_ZTHRESH 0.20
#define DEFAULT_NORMTHRESH 0.90
#define DEFAULT_ALLOWEDCLASS 4.0

#define DEBUG 1

namespace polylidar {


    struct Config
    {
        // 2D base configuration
        int dim = DEFAULT_DIM;
        double alpha = DEFAULT_ALPHA;
        double xyThresh = DEFAULT_XYTHRESH;
        size_t minTriangles = DEFAULT_MINTRIANGLES;
        double minBboxArea = DEFAULT_MINBBOX;
        // 3D configuration
        double zThresh = DEFAULT_ZTHRESH;
        double normThresh = DEFAULT_NORMTHRESH;
        // 4D configuration
        double allowedClass = DEFAULT_ALLOWEDCLASS;
        // double desiredVector[3] = {0, 0, 1};
    };

    struct Polygon {
        std::vector<size_t> shell;
        std::vector<std::vector<size_t>> holes;
    };

    std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>>  _extractPlanesAndPolygons(pybind11::array_t<double> nparray, Config config);

    std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>>  extractPlanesAndPolygons(pybind11::array_t<double> nparray, int dim,
                                double alpha, double xyThresh, size_t minTriangles,
                                double minBboxArea, double zThresh, 
                                double normThresh, double allowedClass);
}


#endif
