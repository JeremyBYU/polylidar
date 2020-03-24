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

#include "Polylidar/Types.hpp"
#include "Polylidar/Utility.hpp"
#include "Polylidar/Mesh/MeshHelper.hpp"
#include "Polylidar/Delaunator/Delaunator.hpp"

#define PL_DEFAULT_DIM 2
#define PL_DEFAULT_ALPHA 1.0
#define PL_DEFAULT_XYTHRESH 0.0
#define PL_DEFAULT_LMAX 0.0
#define PL_DEFAULT_MINTRIANGLES 20
#define PL_DEFAULT_MINHOLEVERTICES 3
#define PL_DEFAULT_MINBBOX 100.0
#define PL_DEFAULT_ZTHRESH 0.20
#define PL_DEFAULT_NORMTHRESH 0.90
#define PL_DEFAULT_NORMTHRESH_MIN 0.1
#define PL_DEFAULT_ALLOWEDCLASS 4.0
#define PL_DEFAULT_STRIDE 2
#define PL_DEFAULT_CALC_NORMALS true

namespace Polylidar {

class Polylidar3D
{

  public:
    Polylidar3D(const double _alpha = PL_DEFAULT_ALPHA, const double _lmax = PL_DEFAULT_LMAX,
                const size_t _min_triangles = PL_DEFAULT_MINTRIANGLES,
                const size_t _min_hole_vertices = PL_DEFAULT_MINHOLEVERTICES,
                const double _z_thresh = PL_DEFAULT_ZTHRESH, const double _norm_thresh = PL_DEFAULT_NORMTHRESH,
                const double _norm_thresh_min = PL_DEFAULT_NORMTHRESH_MIN);
    std::tuple<MeshHelper::HalfEdgeTriangulation, Planes, Polygons>
    ExtractPlanesAndPolygons(const Matrix<double>& points, const std::array<double, 3> plane_normal);

  protected:
    double alpha;
    double lmax;
    size_t min_triangles;
    size_t min_hole_vertices;
    double z_thresh;
    double norm_thresh;
    double norm_thresh_min;

    Planes ExtractPlanes(MeshHelper::HalfEdgeTriangulation& mesh, std::vector<uint8_t>& tri_set, PlaneData& plane_data);
};

} // namespace Polylidar

#endif