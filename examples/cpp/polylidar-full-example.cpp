
#include <tuple>
#include <iomanip>
#include "Polylidar/Polylidar.hpp"

#include "open3d/Open3D.h"

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

    auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
    sphere->ComputeVertexNormals();
    sphere->PaintUniformColor({0.0, 1.0, 0.0});
    open3d::visualization::DrawGeometries({sphere});

    return 0;
}
