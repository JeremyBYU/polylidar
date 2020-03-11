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
#include <ratio>
#include <limits>

#if defined(_OPENMP)
#include <omp.h>
#define PL_OMP_CHUNK_SIZE_TRISET 64
#define PL_OMP_ELEM_PER_THREAD_TRISET 12800
#endif

#include "polylidar/helper.hpp"
#include "polylidar/Mesh/MeshHelper.hpp"
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
#define DEFAULT_CALC_NORMALS true

#define DEBUG 1

#if defined(__GNUC__)
#define FORCE_O1_OPTIMIZATION __attribute__((optimize("O1")))
#else
#define FORCE_O1_OPTIMIZATION
#endif

// namespace py = pybind11;
const static std::array<double, 3> DEFAULT_DESIRED_VECTOR{{0, 0, 1}};
const static std::array<double, 9> DEFAULT_IDENTITY_RM{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
const static uint8_t ZERO_UINT8 = static_cast<uint8_t>(0);
const static uint8_t ONE_UINT8 = static_cast<uint8_t>(1);
const static uint8_t MAX_UINT8 = static_cast<uint8_t>(255);

namespace polylidar
{
using vvi = std::vector<std::vector<size_t>>;
#ifdef PL_USE_STD_UNORDERED_MAP
template <typename T, typename G>
using unordered_map = std::unordered_map<T, G>;
#else
template <typename T, typename G>
using unordered_map = phmap::flat_hash_map<T, G>;
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
    // extra variables needed for planar extraction
    std::array<double, 3> desiredVector = DEFAULT_DESIRED_VECTOR;
    std::array<double, 9> rotationMatrix = DEFAULT_IDENTITY_RM;
    bool needRotation = false;
    // Unique ID for normal being extracted
    uint8_t normalID = ONE_UINT8;
};

struct Polygon
{
    std::vector<size_t> shell;
    std::vector<std::vector<size_t>> holes;
    // I know this looks crazy
    // but for some reason I need this to allow the polygon to have holes
    // without it I could access the holes, but only ONCE. Then they would be gone
    // i.e in python "polygon.holes" <- Now its dead the next time you acces it
    // So I think this now makes a copy on every access.
    vvi getHoles() const { return holes; }
    void setHoles(vvi x) { holes = x; }
};

std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>> ExtractPlanesAndPolygons(Matrix<double> &nparray, Config config);
std::vector<Polygon> ExtractPolygons(Matrix<double> &nparray, Config config);
std::vector<Polygon> ExtractPolygonsAndTimings(Matrix<double> &nparray, Config config, std::vector<float> &timings);
std::tuple<std::vector<std::vector<size_t>>, std::vector<Polygon>> ExtractPlanesAndPolygonsFromMesh(MeshHelper::TriMesh &triangulation, Config config);
std::vector<Polygon> ExtractPolygonsFromMesh(MeshHelper::TriMesh &triangulation, Config config);

std::vector<std::vector<Polygon>> ExtractPolygonsFromMesh(MeshHelper::TriMesh &triangulation, const Matrix<double> &normals, const Config &config);

std::vector<std::vector<size_t>> extractPlanesSet(MeshHelper::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config);
std::vector<std::vector<size_t>> extractPlanesSet(MeshHelper::TriMesh &delaunay, Matrix<double> &points, Config &config);

inline void UpdateConfigWithRotationInformation(Config &config)
{
    std::array<double, 3> axis;
    double angle;
    // std::cout << "Normal to Extract: " << PL_PRINT_ARRAY(config.desiredVector) << "; Z Axis: " << PL_PRINT_ARRAY(DEFAULT_DESIRED_VECTOR) << std::endl;
    std::tie(axis, angle) = axisAngleFromVectors(config.desiredVector, DEFAULT_DESIRED_VECTOR);
    if (std::abs(angle) > EPS_RADIAN)
    {
        config.needRotation = true;
        config.rotationMatrix = axisAngleToRotationMatrix(axis, angle);
    }
}

inline std::vector<Config> CreateMultipleConfigsFromNormals(const Config &config, const Matrix<double> &normals)
{
    std::vector<Config> configs;
    for (size_t i = 0; i < normals.rows; i++)
    {
        auto copy_config = config;
        // Copy normal information into the new config
        copy_config.desiredVector[0] = normals(i, 0);
        copy_config.desiredVector[1] = normals(i, 1);
        copy_config.desiredVector[2] = normals(i, 2);
        copy_config.normalID = static_cast<uint8_t>(i + 1);
        UpdateConfigWithRotationInformation(copy_config);
        configs.emplace_back(std::move(copy_config));
    }
    return configs;
}

} // namespace polylidar

#endif
