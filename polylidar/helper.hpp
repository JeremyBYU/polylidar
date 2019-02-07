
#ifndef POLYLIDARHELPER
#define POLYLIDARHELPER

#include "delaunator.hpp"
#include "polylidar.hpp"

namespace polylidar {

double circumsribedRadius(size_t t, delaunator::Delaunator &delaunay, pybind11::detail::unchecked_reference<double, 2L> &points);

double norm(double a, double b);

}



#endif
