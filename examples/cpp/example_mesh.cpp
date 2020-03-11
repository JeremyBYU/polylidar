#include <iostream>
#include <sstream>
#include <vector>

#include <polylidar/polylidar.hpp>
#include <polylidar/util.hpp>
#include <benchmark/benchmark.h>
#include <Open3D/IO/ClassIO/ImageIO.h>
#include <Open3D/IO/ClassIO/TriangleMeshIO.h>
#include <Open3D/Geometry/Image.h>
#include <Open3D/Geometry/TriangleMesh.h>

const std::string SPARSE_MESH = "./tests/fixtures/meshes/sparse_basement.ply";

namespace o3d = open3d;
using TriMesh = polylidar::MeshHelper::TriMesh;
using namespace polylidar;


void TestMeshExtraction(TriMesh &tri_mesh, Config &config)
{
    std::vector<std::array<double, 3>> normals = {{-0.0123, -0.0021, -0.9999}, {-0.2241,  0.9744, -0.0171}};
    const polylidar::Matrix<double> normals_mat((double *)(normals.data()), 2, 3);
    auto polygons = polylidar::ExtractPolygonsFromMesh(tri_mesh, normals_mat, config);
}

int main(int argc, char const *argv[])
{
    Eigen::Matrix3d mesh_rotation;
    // Rotation for mesh, camera axis to global frame
    mesh_rotation << 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
        0.0, -1.0, 0.0;
    // Sparse Mesh In O3D Format
    std::shared_ptr<o3d::geometry::TriangleMesh> sparse_mesh_o3d = o3d::io::CreateMeshFromFile(SPARSE_MESH);
    sparse_mesh_o3d->Rotate(mesh_rotation);

    auto triangles_ptr = reinterpret_cast<int *>(sparse_mesh_o3d->triangles_.data());
    auto num_triangles = sparse_mesh_o3d->triangles_.size();
    auto vertices_ptr = reinterpret_cast<double *>(sparse_mesh_o3d->vertices_.data());
    auto num_vertices = sparse_mesh_o3d->vertices_.size();
    TriMesh sparse_mesh = polylidar::MeshHelper::CreateTriMeshCopy(vertices_ptr, num_vertices, triangles_ptr, num_triangles);
    polylidar::Config config{3, 0.0, 0.0, 0.10, 1000, 6, 0.0, 0.03, 0.95, 0.90, 1.0, {{0, 0, 1}}};

    TestMeshExtraction(sparse_mesh, config);


    return 0;
}
