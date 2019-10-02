// MIT License

// Copyright (c) 2018 Volodymyr Bilonenko

// Note that this has been heavily modified by Jeremy Castagno

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#ifndef DELAUNATOR
#define DELAUNATOR


#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#ifdef USE_ROBUST
#include "predicates.h"
#endif

#include "util.hpp"

#ifdef PY_EXTENSION
#include "pybind11/pybind11.h" // Pybind11 import to define Python bindings
#include "pybind11/stl.h"      // Pybind11 import for STL containers
#include "pybind11/numpy.h"
#endif

namespace delaunator {
    class Delaunator {

    public:
        polylidar::Matrix coords;
        std::vector<std::size_t> triangles;
        std::vector<std::size_t> halfedges;
        std::vector<std::size_t> hull_prev;
        std::vector<std::size_t> hull_next;
        std::vector<std::size_t> hull_tri;
        std::size_t hull_start;

        Delaunator(polylidar::Matrix &in_coords);
        #ifdef PY_EXTENSION
        Delaunator(pybind11::array_t<double> nparray);
        #endif
        void triangulate();

        double get_hull_area();

    private:
        std::vector<std::size_t> m_hash;
        double m_center_x;
        double m_center_y;
        std::size_t m_hash_size;
        std::vector<std::size_t> m_edge_stack;

        std::size_t legalize(std::size_t a);
        std::size_t hash_key(double x, double y) const;
        std::size_t add_triangle(
            std::size_t i0,
            std::size_t i1,
            std::size_t i2,
            std::size_t a,
            std::size_t b,
            std::size_t c);
        void link(std::size_t a, std::size_t b);
    };

}
#endif