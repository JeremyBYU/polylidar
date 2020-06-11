#ifndef POLYLIDAR_CORE
#define POLYLIDAR_CORE
#include <queue>

#include "Polylidar/Types.hpp"
#include "Polylidar/Utility.hpp"
#include "Polylidar/Mesh/MeshHelper.hpp"

namespace Polylidar {

namespace Core {



void ExtractMeshSet(MeshHelper::HalfEdgeTriangulation& mesh, std::vector<uint8_t>& tri_set, size_t seed_idx,
                    std::vector<size_t>& candidates, const PlaneData &plane_data, const double &z_thresh);
void ConstructPointHash(VUI& plane, MeshHelper::HalfEdgeTriangulation& mesh, PointHash& point_hash, EdgeSet& edge_hash,
                        ExtremePoint& xPoint, PlaneData& plane_data);

Polygons ExtractConcaveHulls(Planes &planes, MeshHelper::HalfEdgeTriangulation &mesh, PlaneData &plane_data, size_t min_hole_vertices_);
Polygon ExtractConcaveHull(VUI &plane, MeshHelper::HalfEdgeTriangulation &mesh, PlaneData &plane_data, size_t min_hole_vertices_);


// Inline Function



inline bool PassPlaneConstraints(std::vector<size_t>& plane_set, size_t min_triangles)
{
    return plane_set.size() >= min_triangles;
}

inline size_t fast_mod(const size_t i, const size_t c) { return i >= c ? i % c : i; }

inline size_t nextHalfedge(size_t e) { return fast_mod(e, 3) == 2 ? e - 2 : e + 1; }

inline std::array<double, 2> GetVector(size_t edge, MeshHelper::HalfEdgeTriangulation& mesh, std::array<double, 9>& rm,
                                       bool& need_rotation, bool flip = false)
{
    auto& coords = mesh.vertices;
    auto& triangles = mesh.triangles.data;
    auto pi = triangles[edge];
    auto piNext = triangles[nextHalfedge(edge)];

    // Points projected on 2D plane, assumes normal is [0,0,1]
    std::array<double, 2> p0 = {coords(pi, 0), coords(pi, 1)};
    std::array<double, 2> p1 = {coords(piNext, 0), coords(piNext, 1)};
    std::array<double, 2> result;

    // Check if points need to be projected onto a different plane
    if (need_rotation)
    {
        // std::cout<< "rotating points for edge: " << edge << std::endl;
        // print_matrix(rm);
        auto rotated_point = Utility::Math::RotateVector(&coords(pi, 0), rm);
        // std::cout<< "p0 before: " << PL_PRINT_ARRAY2(p0) << std::endl;
        p0[0] = rotated_point[0];
        p0[1] = rotated_point[1];
        // std::cout<< "p0 after: " << PL_PRINT_ARRAY2(p0) << std::endl;
        auto rotated_point_2 = Utility::Math::RotateVector(&coords(piNext, 0), rm);
        // std::cout<< "p1 before: " << PL_PRINT_ARRAY2(p1) << std::endl;
        p1[0] = rotated_point_2[0];
        p1[1] = rotated_point_2[1];
        // std::cout<< "p1 after: " << PL_PRINT_ARRAY2(p1) << std::endl;
    }

    if (flip)
    {
        result[0] = p0[0] - p1[0];
        result[1] = p0[1] - p1[1];
    }
    else
    {
        result[0] = p1[0] - p0[0];
        result[1] = p1[1] - p0[1];
    }
    return result; // RVO
}


inline size_t GetHullEdge(const std::array<double, 2>& v1, const std::vector<size_t>& outgoingEdges,
                          MeshHelper::HalfEdgeTriangulation& mesh, std::array<double, 9>& rm, bool& need_rotation,
                          const bool is_ccw = false)
{
    std::vector<std::array<double, 2>> otherVectors;
    std::transform(outgoingEdges.begin(), outgoingEdges.end(), std::back_inserter(otherVectors),
                   [&mesh, &rm, &need_rotation](size_t edge) -> std::array<double, 2> {
                       return GetVector(edge, mesh, rm, need_rotation, false);
                   });

    std::vector<double> angleDist;
    std::transform(otherVectors.begin(), otherVectors.end(), std::back_inserter(angleDist),
                   [&v1](std::array<double, 2>& outVector) -> double { return Utility::Math::Get360Angle(v1, outVector); });

    if (is_ccw)
    {
        auto min_pos = std::distance(angleDist.begin(), std::min_element(angleDist.begin(), angleDist.end()));
        return outgoingEdges[min_pos];
    }
    else
    {
        auto max_pos = std::distance(angleDist.begin(), std::max_element(angleDist.begin(), angleDist.end()));
        return outgoingEdges[max_pos];
    }
}

std::vector<size_t> ConcaveSection(PointHash& pointHash, EdgeSet& edgeHash, MeshHelper::HalfEdgeTriangulation& mesh,
                                   size_t startEdge, size_t stopPoint, PlaneData& plane_data);

std::vector<std::vector<size_t>> ExtractInteriorHoles(PointHash& pointHash, EdgeSet& edgeHash,
                                                      MeshHelper::HalfEdgeTriangulation& mesh, PlaneData& plane_data);

} // namespace Core

} // namespace Polylidar

#endif