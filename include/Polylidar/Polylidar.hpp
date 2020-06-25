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

/**
 * This class handles all data inputs: 2D points sets, 3D point clouds, 3D meshes. Will extract polygons from data.
 * The only issue is the '_alpha' parameter is only applicable for 2D point sets.
 * TODO - Break up into 2D and 3D classes?
 */
class Polylidar3D
{

  public:
    /**
     * @brief Construct a new Polylidar 3D object.
     *
     * @param _alpha              Maximum circumcircle radius (only applicable for 2D point sets). Leave at 0.0 for 3D
     * Data.
     * @param _lmax               Maximum triangle edge length in a planar triangle segment
     * @param _min_triangles      Minimum number of triangles in a planar triangle segment
     * @param _min_hole_vertices  Minimum number of vertices for an interior hole in a polygon
     * @param _z_thresh           Maximum point to plane distance during region growing (3D only). A value of 0.0 ignores this constraint.
     * @param _norm_thresh        IGNORE - will be deprecated or repurposed (3D only)
     * @param _norm_thresh_min    Minimum value of the dot product between a triangle and surface normal being extracted
     * (3D only)
     * @param _task_threads       Number of threads for task-based parallelism (Marl library)
     */
    Polylidar3D(const double _alpha = PL_DEFAULT_ALPHA, const double _lmax = PL_DEFAULT_LMAX,
                const size_t _min_triangles = PL_DEFAULT_MINTRIANGLES,
                const size_t _min_hole_vertices = PL_DEFAULT_MINHOLEVERTICES,
                const double _z_thresh = PL_DEFAULT_ZTHRESH, const double _norm_thresh = PL_DEFAULT_NORMTHRESH,
                const double _norm_thresh_min = PL_DEFAULT_NORMTHRESH_MIN,
                const int _task_threads = PL_DEFAULT_TASK_THREADS);

    /**
     * @brief Extract Planes and Polygons from a 2D point sets or unorganized 3D point clouds
     * Uses 2D Delaunay triangulation for 2D point sets. Uses 2.5 Delaunay triangulation for 3D point clouds.
     * Only send 3D point clouds whose desired surface for extraction is already aligned with the XY Plane (e.g.
     * airborne LiDAR point clouds)
     *
     * @param points            Matrix (NX2 or NX3) point cloud. Contiguous Memory.
     * @param plane_normal      Leave this at the default. It can technically be changed but don't deviate to greatly
     * from [0,0,1].
     * @return std::tuple<MeshHelper::HalfEdgeTriangulation, Planes, Polygons>
     */
    std::tuple<MeshHelper::HalfEdgeTriangulation, Planes, Polygons>
    ExtractPlanesAndPolygons(const Matrix<double>& points,
                             const std::array<double, 3> plane_normal = PL_DEFAULT_DESIRED_VECTOR);

    /**
     * @brief Extracts planes and polygons from a half-edge triangular mesh given a plane normal
     *
     * @param mesh              A half-edge mesh previously created.
     * @param plane_normal      The unit plane normal to extract planes and polygons from the mesh.
     * @return std::tuple<Planes, Polygons>
     */
    std::tuple<Planes, Polygons> ExtractPlanesAndPolygons(MeshHelper::HalfEdgeTriangulation& mesh,
                                                          const std::array<double, 3> plane_normal);
    /**
     * @brief Extracts planes and polygons from a half-edge triangular mesh and makes use of task-based parallelism
     *
     * @param mesh              A half-edge mesh previously created.
     * @param plane_normal      The unit plane normal to extract planes and polygons from the mesh.
     * @return std::tuple<Planes, Polygons>
     */
    std::tuple<Planes, Polygons> ExtractPlanesAndPolygonsOptimized(MeshHelper::HalfEdgeTriangulation& mesh,
                                                                   const std::array<double, 3> plane_normal);

    /**
     * @brief Extracts planes and polygons from a half-edge triangular mesh given multiple dominant plane normals.
     *
     * @param mesh              A half-edge mesh previously created.
     * @param plane_normals     The set of dominant plane normal in the mesh to extract planes and polygons from. Size
     * LX3, L=number of dominant planes.
     * @return std::tuple<PlanesGroup, PolygonsGroup>
     */
    std::tuple<PlanesGroup, PolygonsGroup> ExtractPlanesAndPolygons(MeshHelper::HalfEdgeTriangulation& mesh,
                                                                    const Matrix<double>& plane_normals);

    /**
     * @brief Extracts planes and polygons from a half-edge triangular mesh given multiple dominant plane normals. Uses
     * task-based parallelism.
     *
     * @param mesh              A half-edge mesh previously created.
     * @param plane_normals     The set of dominant plane normal in the mesh to extract planes and polygons from. Size
     * LX3, L=number of dominant planes.
     * @return std::tuple<PlanesGroup, PolygonsGroup>
     */
    std::tuple<PlanesGroup, PolygonsGroup> ExtractPlanesAndPolygonsOptimized(MeshHelper::HalfEdgeTriangulation& mesh,
                                                                             const Matrix<double>& plane_normals);

    /**
     * @brief Extract the triangle set (dominant planar grouping) for each triangle. This is used for
     * debugging/visualization.
     *
     * @param mesh
     * @param plane_normals
     * @return std::vector<uint8_t>
     */
    std::vector<uint8_t> ExtractTriSet(MeshHelper::HalfEdgeTriangulation& mesh, const Matrix<double>& plane_normals);

  public:
    /** @brief Maximum circumcircle radius of a triangle. Filters 'big' triangles. Only applicable for 2D point sets. A
     * value of 0.0 makes this parameter ignored. */
    double alpha;
    /** @brief Maximum triangle edge length of a triangle in a planar segment. Filters 'big' triangles */
    double lmax;
    /** @brief Minimum number of triangles in a planar triangle segment. Filters small planar segments very fast. */
    size_t min_triangles;
    /** @brief Minimum number of vertices for a hole in a polygon. Filters small holes very fast. */
    size_t min_hole_vertices;
    /** @brief Maximum point to plane distance during region growing (3D only). Forces planarity constraints. A value of
     * 0.0 ignores this constraint. */
    double z_thresh;
    /** @brief IGNORE - will be deprecated or repurposed (3D only) */
    double norm_thresh;
    /** @brief Minimum value of the dot product between a triangle and surface normal being extracted. Forces Planar
     * Constraints */
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

    void CreateTriSet3Optimized(std::vector<uint8_t>& tri_set, MeshHelper::HalfEdgeTriangulation& mesh,
                                PlaneData& plane_data);
    void CreateTriSet3OptimizedForMultiplePlanes(std::vector<uint8_t>& tri_set, MeshHelper::HalfEdgeTriangulation& mesh,
                                                 std::vector<PlaneData>& plane_data_list);
};

} // namespace Polylidar

#endif