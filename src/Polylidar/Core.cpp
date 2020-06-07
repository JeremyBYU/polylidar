#include "Polylidar/Core.hpp"

namespace Polylidar {

namespace Core {

constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();


Polygon ExtractConcaveHull(VUI &plane, MeshHelper::HalfEdgeTriangulation& mesh, PlaneData& plane_data,
                                        size_t min_hole_vertices_)
{
    Polygon poly;
    // point hash map
    unordered_map<size_t, std::vector<size_t>> pointHash;
    // hash of all empty border half edges
    unordered_map<size_t, size_t> edgeHash;
    // the left and right most extreme points
    ExtremePoint xPoint;

    // 99.6% of the time inside extractConcaveHull is inside constructPointHash
    // auto before = std::chrono::high_resolution_clock::now();
    Core::ConstructPointHash(plane, mesh, pointHash, edgeHash, xPoint, plane_data);
    // auto after = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "ConstructPointHash - Total (ms): " << elapsed.count() << std::endl;

    // std::vector<size_t> bucket_sizes;
    // for(auto i = 0; i < edgeHash.bucket_count(); ++i)
    // {
    //   auto bucket_size = edgeHash.bucket_size(i);
    //   bucket_sizes.push_back(bucket_size);
    // }
    // std::cout << bucket_sizes << std::endl;

    auto startingHalfEdge = xPoint.xr_he;
    // Error checking, just in case the extreme right point has a hole connected to it
    auto& nextEdges = pointHash[xPoint.xr_pi];
    if (nextEdges.size() > 1)
    {
        // TODO check counter clockwise, need to modify function argument
        startingHalfEdge =
            Core::GetHullEdge(UP_VECTOR, nextEdges, mesh, plane_data.rotation_matrix, plane_data.need_rotation, false);
    }
    auto startingPointIndex = xPoint.xr_pi;
    // std::cout << "Staring point index " << startingPointIndex << std::endl;;
    auto stopPoint = startingPointIndex;
    auto shell = Core::ConcaveSection(pointHash, edgeHash, mesh, startingHalfEdge, stopPoint, plane_data, false);
    auto holes = Core::ExtractInteriorHoles(pointHash, edgeHash, mesh, plane_data);

    holes.erase(
        std::remove_if(holes.begin(), holes.end(),
                       [&min_hole_vertices_](std::vector<size_t>& hole) { return hole.size() < min_hole_vertices_; }),
        holes.end());

    // std::vector<std::vector<size_t>> holes;
    poly.shell = std::move(shell);
    poly.holes = std::move(holes);
    return poly;
}

Polygons ExtractConcaveHulls(Planes &planes, MeshHelper::HalfEdgeTriangulation& mesh, PlaneData& plane_data,
                                          size_t min_hole_vertices_)
{

    std::vector<Polygon> polygons;
    for (auto&& plane : planes)
    {
        Polygon poly = ExtractConcaveHull(plane, mesh, plane_data, min_hole_vertices_);
        polygons.push_back(std::move(poly));
    }
    return polygons;
}

inline void UpdateAverage(double *old_average, const double *new_point, int samples)
{   
    old_average[0] = old_average[0] + (new_point[0] - old_average[0]) / samples;
    old_average[1] = old_average[1] + (new_point[1] - old_average[1]) / samples;
    old_average[2] = old_average[2] + (new_point[2] - old_average[2]) / samples;
}

// need unit normal, and z_thresh
void ExtractMeshSet(MeshHelper::HalfEdgeTriangulation &mesh, std::vector<uint8_t> &tri_set, size_t seed_idx, std::vector<size_t> &candidates, const PlaneData &plane_data, const double &z_thresh)
{
    // Construct queue for triangle neighbor expansion
    std::queue<size_t> queue;
    // Average Point of Planar Segment
    std::array<double, 3> avg_point {{0.0, 0.0, 0.0}};
    // Number of sample for running average 
    int samples = 1;
    // Temporary Variables
    std::array<double, 3> temp_point {{0.0, 0.0, 0.0}};
    double point_to_plane = 0.0;

    // Add seed index to queue and "erase" from tri set
    queue.push(seed_idx);
    tri_set[seed_idx] = MAX_UINT8;

    // aliases
    auto &halfedges = mesh.halfedges;
    auto &vertices = mesh.vertices;
    auto &triangles = mesh.triangles;

    // Average point of of planar segment (from seed idx three vetices)
    avg_point[0] = (vertices(triangles(seed_idx, 0), 0) + vertices(triangles(seed_idx, 1), 0)  + vertices(triangles(seed_idx, 2), 0)) / 3.0;
    avg_point[1] = (vertices(triangles(seed_idx, 0), 1) + vertices(triangles(seed_idx, 1), 1)  + vertices(triangles(seed_idx, 2), 1)) / 3.0;
    avg_point[2] = (vertices(triangles(seed_idx, 0), 2) + vertices(triangles(seed_idx, 1), 2)  + vertices(triangles(seed_idx, 2), 2)) / 3.0;

    // A user may set z_thresh to 0, indicating they are not interested in point to plane distance
    // Put this conditional outside of the while loop
    // If put inside the while loop a slowdown occurs (compiler did not optimize the constant result unfortunately)
    if (z_thresh > 0.0)
    {
        // Calculate point to plane distance constraints during region growing
        while (!queue.empty())
        {
            auto tri = queue.front();
            queue.pop();
            candidates.push_back(tri);
            // Get all neighbors that are inside our triangle hash map
            // Loop through every edge of this triangle and get adjacent triangle of this edge
            for (size_t i = 0; i < 3; ++i)
            {
                auto e = tri * 3 + i;
                auto opposite = halfedges(e);
                if (opposite != Utility::INVALID_INDEX)
                {
                    // convert opposite edge to a triangle
                    size_t tn = opposite / 3;
                    if (tri_set[tn] == plane_data.normal_id)
                    {
                        // Ensure point to plane distance meets minimum requirements
                        Utility::Math::Subtract(&avg_point[0], &vertices(triangles(tn, 0), 0), temp_point);
                        point_to_plane = std::abs(Utility::Math::DotProduct3(plane_data.plane_normal, temp_point));
                        if (point_to_plane < z_thresh)
                        {
                            // Push triangle onto planar segment
                            queue.push(tn);
                            tri_set[tn] = MAX_UINT8;
                            // update rolling average
                            // TODO maybe just stick to the seed point?
                            samples++;
                            UpdateAverage(&avg_point[0], &vertices(triangles(tn, 0), 0), samples);
                        }
                    }
                }
            }
        }
    }
    else
    {
        // Do NOT calculate point to plane distance constraints during region growing
        while (!queue.empty())
        {
            auto tri = queue.front();
            queue.pop();
            candidates.push_back(tri);
            // Get all neighbors that are inside our triangle hash map
            // Loop through every edge of this triangle and get adjacent triangle of this edge
            for (size_t i = 0; i < 3; ++i)
            {
                auto e = tri * 3 + i;
                auto opposite = halfedges(e);
                if (opposite != Utility::INVALID_INDEX)
                {
                    // convert opposite edge to a triangle
                    size_t tn = opposite / 3;
                    if (tri_set[tn] == plane_data.normal_id)
                    {
                        // Push triangle onto planar segment
                        queue.push(tn);
                        tri_set[tn] = MAX_UINT8;
                    }
                }
            }
        }
    }
    
}

inline void TrackExtremePoint(size_t pi, Matrix<double> &points, ExtremePoint &exPoint, size_t he, std::array<double, 9> &rm, bool &need_rotation)
{
    double x_val = points(pi, 0);
    if (need_rotation)
    {
        auto rotated_point = Utility::Math::RotateVector(&points(pi, 0), rm);
        x_val = rotated_point[0];
    }

    if (x_val > exPoint.xr_val)
    {
        exPoint.xr_he = he;
        exPoint.xr_pi = pi;
        exPoint.xr_val = x_val;
    }
}


void ConstructPointHash(VUI &plane, MeshHelper::HalfEdgeTriangulation &mesh, PointHash &point_hash, EdgeSet &edge_hash,
                        ExtremePoint &xPoint, PlaneData &plane_data)
{
    auto &triangles = mesh.triangles;
    auto &halfedges = mesh.halfedges;
    auto &points = mesh.vertices;

    size_t max_triangles_all = static_cast<size_t>(triangles.rows);
    std::vector<uint8_t> triSet(max_triangles_all, false);

    size_t max_triangles = static_cast<size_t>(plane.size());

    // This does not seem to make much of a difference
    // But it does not hurt it, Perimeter (boundary edges) grows as sqrt of area (triangles)
    size_t nominal_edges = static_cast<size_t>(std::sqrt(max_triangles) * 3);
    point_hash.reserve(nominal_edges);
    edge_hash.reserve(nominal_edges);

    // auto before = std::chrono::high_resolution_clock::now();
    // create a hash of all triangles in this plane set
    for (auto &&t : plane)
    {
        triSet[t] = ONE_UINT8;
    }
    // auto after = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "ConstructPointHash - Time creating triangle Hash (ms): " << elapsed.count() << std::endl;

    // Loop through every triangle in the plane
    for (auto &&t : plane)
    {
        // Loop through every edge in the triangle
        for (int i = 0; i < 3; i++)
        {
            // get halfedge index
            auto heIndex = t * 3 + i;
            // get the adjacent edge of this triangle edge
            auto oppHe = halfedges(heIndex);
            // if (oppHe == INVALID_INDEX)
            //     continue;
            size_t oppT = static_cast<size_t>(oppHe / 3);
            // check if this triangle (oppT) is on the convex hull or removed
            if (oppHe == INVALID_INDEX || !triSet[oppT])
            {
                // Record this edge
                edge_hash[heIndex] = heIndex;
                // get point index of this half edge, this is an edge leaving from this pi
                auto pi = triangles(heIndex);
                TrackExtremePoint(pi, points, xPoint, heIndex, plane_data.rotation_matrix, plane_data.need_rotation);
                // Check if the point has already been indexed
                if (point_hash.find(pi) == point_hash.end())
                {
                    // construct a new vector holding this half edge index
                    // point_hash.insert(std::make_pair(pi, std::vector<size_t>(1, heIndex))); // No improvement
                    point_hash[pi] = std::vector<size_t>(1, heIndex);
                }
                else
                {
                    // point already exists, just append to it
                    point_hash[pi].push_back(heIndex);
                }
            }
        }
    }
}

std::vector<size_t> ConcaveSection(PointHash& pointHash, EdgeSet& edgeHash, MeshHelper::HalfEdgeTriangulation& mesh,
                                   size_t startEdge, size_t stopPoint, PlaneData& plane_data, bool isHole)
{

    // std::cout << "Inside concave section" <<std::endl;
    std::vector<size_t> hullSection;
    auto& triangles = mesh.triangles;
    // auto &coords = delaunay.coords;
    auto workingEdge = startEdge;
    // std::cout<< "Starting working edge: " << workingEdge << std::endl;
    while (true)
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // std::cout << "Start while" << std::endl;
        edgeHash.erase(workingEdge);
        // Get the next EDGE of the SAME triangle
        auto nextHalfEdge = nextHalfedge(workingEdge);
        // std::cout << "nextHalf edge: " <<  nextHalfEdge <<std::endl;
        // Get the starting point of this edge
        // auto nextPi = triangles[nextHalfEdge];
        auto nextPi = triangles(nextHalfEdge);
        // std::cout<< "nextPi: " << nextPi << std::endl;
        // std::cout<< "nextPi Coord: " << coords[2 * nextPi] << ", " << coords[2 * nextPi + 1] << std::endl;
        // Add point to the hull section
        hullSection.push_back(nextPi);
        // Check if this is the stop point
        if (nextPi == stopPoint)
        {
            return hullSection;
        }

        // Get outgoing edges for this point
        auto& nextEdges = pointHash[nextPi];

        if (nextEdges.size() == 0)
        {
            std::cerr << "ERROR! Found a broken edge when extracting a concave section (most likely during hole "
                         "extraction). Possible that delaunator mislabeled an edge as part of the convex hull"
                      << std::endl;
            // throw "ERROR! Found a broken edge when extracting a concave section (most likely during hole extraction). Possible that delaunator mislabeled an edge as part of the convex hull";
            // return empty hull
            return std::vector<size_t>();
        }

        // std::cout<< "nextEdges: " << nextEdges << std::endl;

        // filter edges that have already been seen!
        nextEdges.erase(std::remove_if(nextEdges.begin(), nextEdges.end(),
                                       [&edgeHash](size_t& e) { return edgeHash.count(e) == 0; }),
                        nextEdges.end());
        // std::cout<< "nextEdges after filter: " << nextEdges << std::endl;
        if (nextEdges.size() == 1)
        {
            workingEdge = nextEdges[0];
        }
        else
        {
            // We have a junction of outgoing edges at this point
            // std::cout << "Multiple Outgoing Edges. At Working Edge: " << workingEdge << "; At nextPi: " << nextPi <<
            // std::endl;
            auto workingEdgeVector =
                GetVector(workingEdge, mesh, plane_data.rotation_matrix, plane_data.need_rotation, true);
            // std::cout << "Working Edge Vector " << PL_PRINT_ARRAY2(workingEdgeVector) << std::endl;
            workingEdge = GetHullEdge(workingEdgeVector, nextEdges, mesh, plane_data.rotation_matrix,
                                      plane_data.need_rotation, isHole);
            // workingEdge = newEdge;
            // std::cout << "New Edge: " << newEdge << "; Next PI will be: " << triangles[newEdge] << std::endl;
        }
    }

    return hullSection;
}

std::vector<std::vector<size_t>> ExtractInteriorHoles(PointHash& pointHash, EdgeSet& edgeHash,
                                                      MeshHelper::HalfEdgeTriangulation& mesh, PlaneData& plane_data)
{
    std::vector<std::vector<size_t>> allHoles;
    auto& triangles = mesh.triangles;
    // std::cout<< "Starting extracting Interior Holes" << std::endl;
    while (true)
    {
        if (edgeHash.empty())
        {
            break;
        }
        auto startEdge = std::begin(edgeHash)->first;
        // auto startingPointIndex = triangles[startEdge];
        auto stopPoint = triangles(startEdge);
        auto hole = ConcaveSection(pointHash, edgeHash, mesh, startEdge, stopPoint, plane_data, false);
        if (hole.size() > 0)
        {
            allHoles.push_back(hole);
        }
    }

    return allHoles;
}

} // namespace Core

} // namespace Polylidar