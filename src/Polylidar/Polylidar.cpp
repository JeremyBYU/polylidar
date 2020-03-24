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
Polylidar3D::ExtractPlanesAndPolygons(const Matrix<double> points)
{
    Delaunator::Delaunator delaunay(points);
    delaunay.triangulate();

    // Planes planes = extractPlanesSet(delaunay, nparray, config);


    // // before = std::chrono::high_resolution_clock::now();
    // std::vector<Polygon> polygons = extractConcaveHulls(planes, delaunay, nparray, config);
    // // after = std::chrono::high_resolution_clock::now();
    // // elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // // std::cout << "Polygon Hull Extraction took " << elapsed.count() << " milliseconds" << std::endl;
    // return std::make_tuple(delaunay, planes, polygons);
    
}

} // namespace Polylidar