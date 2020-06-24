// MIT License

// Copyright (c) 2020 Jeremy Castagno

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
#include "Polylidar/Core.hpp"

#include "marl/scheduler.h"

#if defined(_OPENMP)
#include <omp.h>
#define PL_OMP_CHUNK_SIZE_TRISET 64
#define PL_OMP_ELEM_PER_THREAD_TRISET 12800
#endif

namespace Polylidar {

class Polylidar3D
{

  public:
    Polylidar3D(const double _alpha = PL_DEFAULT_ALPHA, const double _lmax = PL_DEFAULT_LMAX,
                const size_t _min_triangles = PL_DEFAULT_MINTRIANGLES,
                const size_t _min_hole_vertices = PL_DEFAULT_MINHOLEVERTICES,
                const double _z_thresh = PL_DEFAULT_ZTHRESH, const double _norm_thresh = PL_DEFAULT_NORMTHRESH,
                const double _norm_thresh_min = PL_DEFAULT_NORMTHRESH_MIN,
                const int _task_threads = PL_DEFAULT_TASK_THREADS);
    std::tuple<MeshHelper::HalfEdgeTriangulation, Planes, Polygons>
    ExtractPlanesAndPolygons(const Matrix<double>& points, const std::array<double, 3> plane_normal = PL_DEFAULT_DESIRED_VECTOR);
    std::tuple<Planes, Polygons> ExtractPlanesAndPolygons(MeshHelper::HalfEdgeTriangulation& mesh,
                                                          const std::array<double, 3> plane_normal);

    std::tuple<Planes, Polygons> ExtractPlanesAndPolygonsOptimized(MeshHelper::HalfEdgeTriangulation& mesh,
                                                                   const std::array<double, 3> plane_normal);

    std::tuple<PlanesGroup, PolygonsGroup> ExtractPlanesAndPolygons(MeshHelper::HalfEdgeTriangulation& mesh,
                                                                    const Matrix<double>& plane_normals);
    std::tuple<PlanesGroup, PolygonsGroup> ExtractPlanesAndPolygonsOptimized(MeshHelper::HalfEdgeTriangulation& mesh,
                                                                             const Matrix<double>& plane_normals);

    std::vector<uint8_t> ExtractTriSet(MeshHelper::HalfEdgeTriangulation& mesh, const Matrix<double>& plane_normals);

  public:
    double alpha;
    double lmax;
    size_t min_triangles;
    size_t min_hole_vertices;
    double z_thresh;
    double norm_thresh;
    double norm_thresh_min;

  private:
    int task_threads;

    std::shared_ptr<marl::Scheduler> scheduler;

  private:
    Planes ExtractPlanes(MeshHelper::HalfEdgeTriangulation& mesh, std::vector<uint8_t>& tri_set, PlaneData& plane_data,
                         bool tri_set_finished = false);
    std::tuple<Planes, Polygons> ExtractPlanesWithTasks(MeshHelper::HalfEdgeTriangulation& mesh,
                                                        std::vector<uint8_t>& tri_set, PlaneData& plane_data);
    void CreateTriSet2(std::vector<uint8_t>& tri_set, MeshHelper::HalfEdgeTriangulation& mesh);
    // void CreateTriSet3(std::vector<uint8_t>& tri_set, MeshHelper::HalfEdgeTriangulation& mesh, PlaneData& plane_data);
    void CreateTriSet3Optimized(std::vector<uint8_t>& tri_set, MeshHelper::HalfEdgeTriangulation& mesh,
                                PlaneData& plane_data);
    void CreateTriSet3OptimizedForMultiplePlanes(std::vector<uint8_t>& tri_set, MeshHelper::HalfEdgeTriangulation& mesh,
                                                 std::vector<PlaneData>& plane_data_list);
};

} // namespace Polylidar

#endif