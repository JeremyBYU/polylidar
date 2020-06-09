
#include "Polylidar/Polylidar.hpp"
#include "Polylidar/Delaunator/Delaunator.hpp"
#include "npy/npy.h"

#include <Open3D/IO/ClassIO/TriangleMeshIO.h>
#include <Open3D/Geometry/TriangleMesh.h>
#include <omp.h>

using namespace Polylidar;
namespace o3d = open3d;
using TriMesh = MeshHelper::HalfEdgeTriangulation;

void PrintPoint(Polylidar::Delaunator::Delaunator &mesh, size_t idx)
{

    std:: cout << "[" << mesh.triangles(idx, 0) << ", " << mesh.triangles(idx, 1) << ", " << mesh.triangles(idx, 2) << "]" <<  std::endl;
}

int main(int argc, char const *argv[])
{
    const std::string SPARSE_MESH = "./fixtures/meshes/sparse_basement.ply";
    Eigen::Matrix3d mesh_rotation;
    std::shared_ptr<o3d::geometry::TriangleMesh> sparse_mesh_o3d;

    // Rotation for mesh, camera axis to global frame
    mesh_rotation << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
    // Sparse Mesh In O3D Format
    sparse_mesh_o3d = o3d::io::CreateMeshFromFile(SPARSE_MESH);
    sparse_mesh_o3d->Rotate(mesh_rotation);
    // sparse_mesh_o3d->Rotate();
    // Sparse Mesh In HalfEdgeTriangulation
    auto triangles_ptr = reinterpret_cast<int*>(sparse_mesh_o3d->triangles_.data());
    auto num_triangles = sparse_mesh_o3d->triangles_.size();
    auto vertices_ptr = reinterpret_cast<double*>(sparse_mesh_o3d->vertices_.data());
    auto num_vertices = sparse_mesh_o3d->vertices_.size();
    Matrix<int> triangles(triangles_ptr, num_triangles, 3);
    Matrix<double> vertices(vertices_ptr, num_vertices, 3);

    TriMesh sparse_mesh = MeshHelper::CreateTriMeshCopy(vertices, triangles, true);

    // MeshHelper::BilateralFilterNormals(sparse_mesh, 1, 0.1, 0.1);
    // alpha, lmax, min_tri, min_hole_vert, z_thresh, norm__thresh, norm_thresh_min, _task_threads
    Polylidar3D pl3d(0.0, 0.1, 1000, 6, 0.03, 0.95, 0.90);
    auto planes_and_polygons = pl3d.ExtractPlanesAndPolygonsOptimized(sparse_mesh, {{0.0, 0.0, 1.0}});
    std::cout << "Stop Here";


    return 0;
}
