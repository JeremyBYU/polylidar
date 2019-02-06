#ifndef DELAUNATORP
#define DELAUNATORP


#include "pybind11/pybind11.h" // Pybind11 import to define Python bindings
#include "pybind11/stl.h"      // Pybind11 import for STL containers
#include "pybind11/numpy.h"

#include "delaunator.hpp"

namespace delaunator {

class DelaunatorP {

public:
    DelaunatorP(pybind11::array_t<double> nparray);

};

}

#endif