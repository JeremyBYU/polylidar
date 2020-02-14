// MIT License

// Copyright (c) 2018 Jeremy Castagno

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
#ifndef POLYLIDAR
#define POLYLIDAR
#define _USE_MATH_DEFINES

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

#if defined(_OPENMP)
#include <omp.h>
#define PL_OMP_CHUNK_SIZE_TRISET 64
#define PL_OMP_ELEM_PER_THREAD_TRISET 12800
#define PL_OMP_MAX_THREAD_DEPTH_TO_PC 8
#endif

#include "polylidar/helper.hpp"
#include "delaunator.hpp"
#include <parallel_hashmap/phmap.h>

#define DEFAULT_DIM 2
#define DEFAULT_ALPHA 1.0
#define DEFAULT_XYTHRESH 0.0
#define DEFAULT_LMAX 0.0
#define DEFAULT_MINTRIANGLES 20
#define DEFAULT_MINHOLEVERTICES 3
#define DEFAULT_MINBBOX 100.0
#define DEFAULT_ZTHRESH 0.20
#define DEFAULT_NORMTHRESH 0.90
#define DEFAULT_NORMTHRESH_MIN 0.1
#define DEFAULT_ALLOWEDCLASS 4.0
#define DEFAULT_STRIDE 2
#define EPS_RADIAN 0.001

#define DEBUG 1

#if defined(__GNUC__)
#define FORCE_O1_OPTIMIZATION __attribute__((optimize("O1")))
#else
#define FORCE_O1_OPTIMIZATION 
#endif

// namespace py = pybind11;
const static std::array<double, 3> DEFAULT_DESIRED_VECTOR{{0,0,1}}; 
const static std::array<double, 9> DEFAULT_IDENTITY_RM{{1,0,0,0,1,0,0,0,1}}; 

namespace polylidar {
    using vvi = std::vector<std::vector<size_t>>;
    #ifdef PL_USE_STD_UNORDERED_MAP
    template<typename T, typename G>
    using unordered_map = std::unordered_map<T,G>;
    #else
    template<typename T, typename G>
    using unordered_map = phmap::flat_hash_map<T,G>;
    #endif

    const static std::array<double, 2> UP_VECTOR = {0.0, 1.0};
    const static std::array<double, 2> DOWN_VECTOR = {0.0, -1.0};
    
    struct Config
    {
        // 2D base configuration
        size_t dim = DEFAULT_DIM;
        double alpha = DEFAULT_ALPHA;
        double xyThresh = DEFAULT_XYTHRESH;
        double lmax = DEFAULT_LMAX;
        size_t minTriangles = DEFAULT_MINTRIANGLES;
        size_t minHoleVertices = DEFAULT_MINHOLEVERTICES;
        double minBboxArea = DEFAULT_MINBBOX;
        // 3D configuration
        double zThresh = DEFAULT_ZTHRESH;
        double normThresh = DEFAULT_NORMTHRESH;
        double normThreshMin = DEFAULT_NORMTHRESH_MIN;
        // 4D configuration
        double allowedClass = DEFAULT_ALLOWEDCLASS;
        // extra needed for 3D
        std::array<double, 3> desiredVector = DEFAULT_DESIRED_VECTOR;
        std::array<double, 9> rotationMatrix = DEFAULT_IDENTITY_RM;
        bool needRotation = false;
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


    std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>>  ExtractPlanesAndPolygons(Matrix<double> &nparray, Config config);
    std::vector<Polygon> ExtractPolygons(Matrix<double> &nparray, Config config);
    std::vector<Polygon> ExtractPolygonsAndTimings(Matrix<double> &nparray, Config config, std::vector<float> &timings);
    std::tuple<std::vector<std::vector<size_t>>, std::vector<Polygon>>  ExtractPlanesAndPolygonsFromMesh(delaunator::HalfEdgeTriangulation &triangulation, Config config);
    std::vector<Polygon>  ExtractPolygonsFromMesh(delaunator::HalfEdgeTriangulation &triangulation, Config config);
    std::vector<double> ExtractPointCloudFromFloatDepth(const Matrix<float> &im, const Matrix<double> &intrinsics, const size_t stride);
    std::tuple<std::vector<double>, std::vector<size_t>, std::vector<size_t>> ExtractUniformMeshFromFloatDepth(const Matrix<float> &im, const Matrix<double> &intrinsics, const size_t stride);

}


#endif
