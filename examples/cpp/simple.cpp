
#include <tuple>
#include <iomanip>
#include "Polylidar/Polylidar.hpp"

template <typename TElem>
std::ostream& operator<<(std::ostream& os, const std::vector<TElem>& vec)
{
    auto iter_begin = vec.begin();
    auto iter_end = vec.end();
    os << "[";
    for (auto iter = iter_begin; iter != iter_end; ++iter)
    {
        std::cout << ((iter != iter_begin) ? "," : "") << *iter;
    }
    os << "]";
    return os;
}
int main(int argc, char const* argv[])
{
    std::cout << "Very Simple C++ Example of Polylidar3D on 2D Point Set" << std::endl;
    std::vector<double> points_data = {
        0.0, 0.0, // Point index 0
        0.0, 1.0, // Point index 1
        1.0, 1.0, // Point index 2
        1.0, 0.0, // Point index 3
        5.0, 0.1, // Point index 4, outlier and should not be included in polygon
    };

    // 5 X 2 matrix as one contigious array
    std::vector<std::size_t> shape = {points_data.size() / 2, 2};
    Polylidar::Matrix<double> points(points_data.data(), shape[0], shape[1]);
    // Set configuration parameters

    // alpha, lmax, min triangles, min hole vertices,
    Polylidar::Polylidar3D pl(0.0, 2.0, 1, 3);

    Polylidar::MeshHelper::HalfEdgeTriangulation mesh;
    Polylidar::Planes planes;
    Polylidar::Polygons polygons;
    auto before = std::chrono::high_resolution_clock::now();
std:
    tie(mesh, planes, polygons) = pl.ExtractPlanesAndPolygons(points);
    auto after = std::chrono::high_resolution_clock::now();

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    std::cout << "Polylidar took " << elapsed.count() << " milliseconds processing a " << shape[0] << " point cloud"
              << std::endl;
    std::cout << "Point indices of Polygon Shell: ";
    // Extract polygon
    for (auto const& polygon : polygons)
    {
        std::cout << polygon.shell << std::endl;
    }
    // Point indices of Polygon Shell: [3,0,1,2]

    std::cout << std::endl;

    return 0;
}
