#include "Polylidar/Polylidar.hpp"

namespace Polylidar {

Polylidar3D::Polylidar3D(const double _alpha, const double _lmax, const size_t _min_triangles,
                         const size_t _min_hole_vertices, const double _z_thresh, const double _norm_thresh,
                         const double _norm_thresh_min)
    : alpha(_alpha),
      lmax(_lmax),
      min_triangles(_min_triangles),
      min_hole_vertices(_min_hole_vertices),
      z_thresh(_z_thresh),
      norm_thresh(_norm_thresh),
      norm_thresh_min(_norm_thresh_min)
{
}

std::tuple<MeshHelper::HalfEdgeTriangulation, Planes, Polygons>
Polylidar3D::ExtractPlanesAndPolygons(const Matrix<double>& points, const std::array<double, 3> plane_normal)
{
    Delaunator::Delaunator delaunay(points);
    delaunay.triangulate();

    // Determine whether we w

    // Create Plane Data Structure, informs Polylidar which normal to extract on
    PlaneData plane_data{plane_normal};
    Utility::UpdatePlaneDataWithRotationInformation(plane_data);

    size_t max_triangles = delaunay.triangles.rows;
    std::vector<uint8_t> tri_set(max_triangles, 0);
    auto planes = ExtractPlanes(delaunay, tri_set, plane_data);

    Polygons polygons;
    // Matrix<double> plane_normals;
    // plane_normals.data.push_back(plane_normal[0]); plane_normals.data.push_back(plane_normal[1]);
    // plane_normals.data.push_back(plane_normal[2]); plane_normals.UpdatePtrFromData(1, 3);

    // Planes planes = extractPlanesSet(delaunay, nparray, config);

    // // before = std::chrono::high_resolution_clock::now();
    // std::vector<Polygon> polygons = extractConcaveHulls(planes, delaunay, nparray, config);
    // // after = std::chrono::high_resolution_clock::now();
    // // elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // // std::cout << "Polygon Hull Extraction took " << elapsed.count() << " milliseconds" << std::endl;
    return std::make_tuple(std::move(delaunay), std::move(planes), std::move(polygons));
}

Planes Polylidar3D::ExtractPlanes(MeshHelper::HalfEdgeTriangulation& mesh, std::vector<uint8_t>& tri_set, PlaneData& plane_data)
{
    auto dim = mesh.vertices.cols;

    Planes planes;
    size_t max_triangles = mesh.triangles.rows;

    // createTriSet2(tri_set, delaunay, points, config);

    // for (size_t t = 0; t < max_triangles; t++)
    // {
    //     if (triSet[t] == config.normalID)
    //     {

    //         planes.emplace_back();                       // construct empty vector inside planes
    //         auto &planeMesh = planes[planes.size() - 1]; // retrieve this newly created vector
    //         extractMeshSet(delaunay, tri_set, t, planeMesh, config.normalID);
    //         if (!passPlaneConstraints(planeMesh, config))
    //         {
    //             planes.pop_back();
    //         }
    //     }
    // }

    return planes;
}

} // namespace Polylidar