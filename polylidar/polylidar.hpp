
#ifndef POLYLIDAR
#define POLYLIDAR


#include "helper.hpp"
#include "delaunator.hpp"

#include "pybind11/pybind11.h" // Pybind11 import to define Python bindings
#include "pybind11/stl.h"      // Pybind11 import for STL containers
#include <pybind11/stl_bind.h> // Pybind11 stl bindings
#include "pybind11/numpy.h"

namespace polylidar {
    struct Config
    {
        int dim = 2;
        double alpha = 1.0;
        Config()
        {
            dim = 2;
        }
    }

    // void extractPlanesAndPolygons(pybind11::array_t<double> nparray, config)



}


#endif
