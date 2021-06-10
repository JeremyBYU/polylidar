
#include <tuple>
#include <random>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include "Polylidar/Polylidar.hpp"
#include "FastGA/FastGA.hpp"
#include "vis_util.hpp"
#include "npy/npy.h"
#include "open3d/Open3D.h"

FastGA::MatX3d DownSampleNormals(Polylidar::Matrix<double>& triangle_normals, double down_sample_fraction = 0.20)
{
    int starting_normals = static_cast<int>(triangle_normals.rows);
    int ds_normals = static_cast<int>(static_cast<double>(starting_normals) * down_sample_fraction);

    // Create random sampling
    std::mt19937 rng(0);
    std::uniform_int_distribution<int> gen(0, starting_normals); // uniform, unbiased

    FastGA::MatX3d new_normals(ds_normals);
    for (int i = 0; i < ds_normals; ++i)
    {
        int r = gen(rng);
        new_normals[i] = {triangle_normals(r, 0), triangle_normals(r, 1), triangle_normals(r, 2)};
    }
    return new_normals;
}

Polylidar::Matrix<double> ConvertMatX3dtoMatrix(FastGA::MatX3d& matrix)
{
    Polylidar::Matrix<double> new_matrix(&matrix[0][0], matrix.size(), 3);
    return new_matrix;
}

int main(int argc, char const* argv[])
{
    std::cout << "A more complete example of using Polylidar3D with 3D Data. Needs Open3D and FastGA." << std::endl;

    // Load Point Cloud Data
    std::vector<unsigned long> shape;
    bool fortran_order;
    std::vector<double> data;
    std::cout << "Loading previously captured Organized Point Cloud from an L515 Camera." << std::endl;
    try
    {
        std::cout << "Attempting to load OPC file from ./fixtures/realsense/opc_example_one/L515_OPC.npy" << std::endl;
        npy::LoadArrayFromNumpy("./fixtures/realsense/opc_example_one/L515_OPC.npy", shape, fortran_order, data);
    }
    catch (const std::exception& e)
    {
        try
        {
            std::cout << "Last chance. Trying to load OPC file from ../fixtures/realsense/opc_example_one/L515_OPC.npy"
                      << std::endl;
            npy::LoadArrayFromNumpy("../fixtures/realsense/opc_example_one/L515_OPC.npy", shape, fortran_order, data);
        }
        catch (const std::exception& e)
        {
            std::cout << "Can't find L515_OPC.npy file. Exiting..." << std::endl;
            std::exit(1);
        }
    }

    // Create Matrix Wrapper around Point Cloud Data (no copy)
    std::cout << "Shape of Point Cloud is: " << shape << std::endl;
    auto num_points = shape[0] * shape[1];
    auto cols = shape[2]; // 3
    // Expected to be a num_points X 3 array but **organized* in memory
    Polylidar::Matrix<double> opc(data.data(), num_points, cols);

    // Visualize Point Cloud
    std::cout << "Visualizing Point Cloud. Rotate View. Close Open3D window when satisfied..." << std::endl;
    auto pcd_eigen = VisUtility::MapEigen<Eigen::Vector3d>(opc);
    std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr(new open3d::geometry::PointCloud(pcd_eigen));
    open3d::visualization::DrawGeometries({pointcloud_ptr});

    // Create Mesh From Point Cloud
    std::cout << "Creating Mesh and smoothing. NOT using optimized filters, see OrganizedPointFilters" << std::endl;
    Polylidar::MeshHelper::HalfEdgeTriangulation tri_mesh;
    Polylidar::VUI valid_idx;
    std::tie(tri_mesh, valid_idx) =
        Polylidar::MeshHelper::ExtractTriMeshFromOrganizedPointCloud(opc, shape[0], shape[1], 1, true);
    Polylidar::MeshHelper::LaplacianFilterVertices(tri_mesh, 3, 1.0);
    tri_mesh.ComputeTriangleNormals();
    Polylidar::MeshHelper::BilateralFilterNormals(tri_mesh, 5, 0.2, 0.25);

    // Visualize Mesh
    std::cout << "Visualizing mesh. Rotate View." << std::endl;
    auto o3d_mesh = VisUtility::CreateOpen3DMesh(tri_mesh);
    open3d::visualization::DrawGeometries({o3d_mesh});

    // Find Dominant Plane Normals
    FastGA::GaussianAccumulatorS2Beta fastgac(4);
    auto downsampled_normals = DownSampleNormals(tri_mesh.triangle_normals);
    fastgac.Integrate(downsampled_normals);
    FastGA::MatX3d peaks = fastgac.FindPeaks(50, false, 0.10, 0.15);
    std::cout << "Detected Peaks with FastGaussianAccumulator. Here are the detected peaks: " << peaks << std::endl;

    // Extracting Planes and Polygons
    std::cout << "Extracting Polygons from all detected peaks: " << std::endl;
    Polylidar::Matrix<double> peaks_mat = ConvertMatX3dtoMatrix(peaks);
    Polylidar::Polylidar3D pl(0.0, 0.05, 500, 10, 0.15, 0.95, 0.95);
    Polylidar::PlanesGroup planes_group;
    Polylidar::PolygonsGroup polygons_group;
    std::tie(planes_group, polygons_group) = pl.ExtractPlanesAndPolygons(tri_mesh, peaks_mat);

    // Paint Planes for visualization
    for (auto& plane_group : planes_group)
    {
        for (auto& plane_segment : plane_group)
        {
            auto color = VisUtility::NextColor();
            VisUtility::PaintPlane(o3d_mesh, plane_segment, color);
        }
    }

    // Create Open3D Polygons (LineSets (no thickness))
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> to_draw;
    for (auto& polygon_group : polygons_group)
    {
        for (auto& polygon : polygon_group)
        {
            auto ls = VisUtility::CreateLineSetFromPolygon(polygon, tri_mesh.vertices);
            to_draw.push_back(ls);
        }
    }
    std::cout << "Visualing Planes and Polygon. Each plane segment is color coded. Polgyons are shown as thin lines "
                 "(OpenGL has no thickness): "
              << std::endl;
    to_draw.push_back(o3d_mesh);
    open3d::visualization::DrawGeometries(to_draw);

    return 0;
}
