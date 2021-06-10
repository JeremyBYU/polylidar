
#include <tuple>
#include <iomanip>
#include <random>
#include <algorithm>
#include "Polylidar/Polylidar.hpp"
#include "npy/npy.h"
#include <cmath>
#include <cfloat>
#include "FastGA/FastGA.hpp"

#include "open3d/Open3D.h"


template<typename eigen_vector, typename from_type=double>
std::vector<eigen_vector> MapEigen(Polylidar::Matrix<from_type> &array, const bool set_nan_to_zero=true) {
    std::vector<eigen_vector> eigen_vectors;
    using TO_T = eigen_vector::Scalar;
    for (auto i = 0; i < array.rows; ++i) {
        // Open3D Point cloud can't handle Nans
        if (set_nan_to_zero && std::isnan(static_cast<double>(array(i, 0))))
        {
            // eigen_vector test(static_cast<TO_T>(array(i, 0)), static_cast<TO_T>(array(i, 1)), static_cast<TO_T>(array(i,2)));
            // eigen_vectors.push_back(test);
            eigen_vectors.emplace_back(static_cast<TO_T>(0), static_cast<TO_T>(0), static_cast<TO_T>(0));
        }
        else
        {
           eigen_vectors.emplace_back(static_cast<TO_T>(array(i, 0)), static_cast<TO_T>(array(i, 1)), static_cast<TO_T>(array(i,2))); 
        }
    }
    return eigen_vectors;
}

std::shared_ptr<open3d::geometry::TriangleMesh> CreateOpen3DMesh(Polylidar::MeshHelper::HalfEdgeTriangulation &tri_mesh)
{
    std::shared_ptr<open3d::geometry::TriangleMesh> triangle_ptr(new open3d::geometry::TriangleMesh());
    triangle_ptr->vertices_ = MapEigen<Eigen::Vector3d>(tri_mesh.vertices);
    triangle_ptr->triangles_ = MapEigen<Eigen::Vector3i, size_t>(tri_mesh.triangles, false);
    triangle_ptr->triangle_normals_= MapEigen<Eigen::Vector3d>(tri_mesh.triangle_normals);
    triangle_ptr->ComputeVertexNormals();
    triangle_ptr->PaintUniformColor({0.20, 0.40, 0.91});

    // std::cout << "Vertices:  "  << triangle_ptr->vertices_.size() <<  "; " << triangle_ptr->vertices_[0] << std::endl;
    // std::cout << "Triangles: " << triangle_ptr->triangles_.size() <<  "; " << triangle_ptr->triangles_[0] << std::endl;
    return triangle_ptr;
}

FastGA::MatX3d DownSampleNormals(Polylidar::Matrix<double> &triangle_normals, double down_sample_fraction=0.20)
{
    int starting_normals = static_cast<int>(triangle_normals.rows);
    int ds_normals = static_cast<int>(static_cast<double>(starting_normals) * down_sample_fraction);

    // Create random sampling
    std::mt19937 rng(0);
    std::uniform_int_distribution<int> gen(0, starting_normals); // uniform, unbiased

    FastGA::MatX3d new_normals(ds_normals);
    for(int i = 0; i < ds_normals; ++i)
    {
        int r = gen(rng);
        new_normals[i] = {triangle_normals(r, 0), triangle_normals(r, 1), triangle_normals(r, 2)};
    }

    return new_normals;

}


int main(int argc, char const* argv[])
{
    std::cout << "A more full example of using Polylidar3D with 3D Data. Needs Open3D." << std::endl;
    
    // Load Point Cloud Data
    std::vector<unsigned long> shape;
    bool fortran_order;
    std::vector<double> data;
    std::cout << "Loading previously captured Organized Point Cloud from an L515 Camera." << std::endl;
    npy::LoadArrayFromNumpy("../fixtures/realsense/opc_example_one/L515_OPC.npy", shape, fortran_order, data);
    std::cout << "Shape of Point Cloud is: " << shape << std::endl;

    auto num_points = shape[0] * shape[1];
    auto cols = shape[2]; //3
    // Create Matrix Wrapper around Point Cloud Data (no copy)
    Polylidar::Matrix<double> opc(data.data(), num_points, cols);
    
    // Print Out First Vertex
    // std::cout << "First Vertex: " << opc(0, 0) << ", " << opc(0, 1) << ", " << opc(0, 2) << std::endl;

    // Visualize Point Cloud
    std::cout << "Visualizing Point Cloud. Close Open3D window when satisfied..." << std::endl;
    auto pcd_eigen = MapEigen<Eigen::Vector3d>(opc);
    std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr(new open3d::geometry::PointCloud(pcd_eigen));
    open3d::visualization::DrawGeometries({pointcloud_ptr});

    // Create Mesh From Point Cloud
    Polylidar::MeshHelper::HalfEdgeTriangulation tri_mesh;
    Polylidar::VUI valid_idx;
    std::tie(tri_mesh, valid_idx) = Polylidar::MeshHelper::ExtractTriMeshFromOrganizedPointCloud(opc, shape[0], shape[1], 1, true);
    Polylidar::MeshHelper::LaplacianFilterVertices(tri_mesh, 3, 1.0);
    tri_mesh.ComputeTriangleNormals();
    Polylidar::MeshHelper::BilateralFilterNormals(tri_mesh, 5, 0.2, 0.25);

    // Visualize Mesh
    auto o3d_mesh = CreateOpen3DMesh(tri_mesh);
    open3d::visualization::DrawGeometries({o3d_mesh});

    // Find Dominant Plane Normals
    FastGA::GaussianAccumulatorS2Beta fastgac(4);
    auto downsampled_normals = DownSampleNormals(tri_mesh.triangle_normals);
    fastgac.Integrate(downsampled_normals);
    auto peaks = fastgac.FindPeaks(50, false, 0.10, 0.15);
    std::cout << "Here are the detected peaks: " << peaks << std::endl;

    // Polylidar::Polylidar3D pl(0.0, 0.05, 500, 10, 0.15, 0.95, 0.95);
    // Polylidar::Planes planes;
    // Polylidar::Polygons polygons;
    // auto before = std::chrono::high_resolution_clock::now();
    // std:tie(planes, polygons) = pl.ExtractPlanesAndPolygons(mesh, );

    return 0;
}
