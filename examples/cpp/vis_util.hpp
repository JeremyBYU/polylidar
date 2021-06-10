
#include <vector>
#include "FastGA/FastGA.hpp"
#include "open3d/Open3D.h"
namespace VisUtility {

FastGA::MatX3d colors_palette{{0.12156862745098039, 0.4666666666666667, 0.7058823529411765},
                              {1.0, 0.4980392156862745, 0.054901960784313725},
                              {0.17254901960784313, 0.6274509803921569, 0.17254901960784313},
                              {0.8392156862745098, 0.15294117647058825, 0.1568627450980392},
                              {0.5803921568627451, 0.403921568627451, 0.7411764705882353},
                              {0.5490196078431373, 0.33725490196078434, 0.29411764705882354},
                              {0.8901960784313725, 0.4666666666666667, 0.7607843137254902},
                              {0.4980392156862745, 0.4980392156862745, 0.4980392156862745},
                              {0.7372549019607844, 0.7411764705882353, 0.13333333333333333},
                              {0.09019607843137255, 0.7450980392156863, 0.8117647058823529}};

int color_counter = 0;

std::array<double, 3> NextColor()
{
    auto color = colors_palette[color_counter];
    color_counter++;
    if (color_counter >= colors_palette.size())
    {
        color_counter = 0;
    }
    return color;
}

template <typename eigen_vector, typename from_type = double>
std::vector<eigen_vector> MapEigen(Polylidar::Matrix<from_type>& array, const bool set_nan_to_zero = true)
{
    std::vector<eigen_vector> eigen_vectors;
    using TO_T = eigen_vector::Scalar;
    for (auto i = 0; i < array.rows; ++i)
    {
        // Open3D Point cloud can't handle Nans
        if (set_nan_to_zero && std::isnan(static_cast<double>(array(i, 0))))
        {
            eigen_vectors.emplace_back(static_cast<TO_T>(0), static_cast<TO_T>(0), static_cast<TO_T>(0));
        }
        else
        {
            eigen_vectors.emplace_back(static_cast<TO_T>(array(i, 0)), static_cast<TO_T>(array(i, 1)),
                                       static_cast<TO_T>(array(i, 2)));
        }
    }
    return eigen_vectors;
}

std::shared_ptr<open3d::geometry::TriangleMesh> CreateOpen3DMesh(Polylidar::MeshHelper::HalfEdgeTriangulation& tri_mesh)
{
    std::shared_ptr<open3d::geometry::TriangleMesh> triangle_ptr(new open3d::geometry::TriangleMesh());
    triangle_ptr->vertices_ = MapEigen<Eigen::Vector3d>(tri_mesh.vertices);
    triangle_ptr->triangles_ = MapEigen<Eigen::Vector3i, size_t>(tri_mesh.triangles, false);
    triangle_ptr->triangle_normals_ = MapEigen<Eigen::Vector3d>(tri_mesh.triangle_normals);
    triangle_ptr->ComputeVertexNormals();
    triangle_ptr->PaintUniformColor({0.5, 0.5, 0.5});
    return triangle_ptr;
}

void PaintPlane(std::shared_ptr<open3d::geometry::TriangleMesh> o3d_mesh, std::vector<size_t>& segment,
                std::array<double, 3> color)
{
    auto& triangles = o3d_mesh->triangles_;
    Eigen::Vector3d color_e(color[0], color[1], color[2]);
    for (auto& t : segment)
    {
        auto vertices_idx = triangles[t];
        o3d_mesh->vertex_colors_[vertices_idx[0]] = color_e;
        o3d_mesh->vertex_colors_[vertices_idx[1]] = color_e;
        o3d_mesh->vertex_colors_[vertices_idx[2]] = color_e;
    }
}

std::shared_ptr<open3d::geometry::LineSet> CreateLineSetFromPolygon(Polylidar::Polygon& polygon,
                                                                    Polylidar::Matrix<double> points,
                                                                    Eigen::Vector3d shell_color = {0.0, 1.0, 0.0},
                                                                    Eigen::Vector3d hole_color = {1.0, 0.0, 0.0})
{
    std::vector<Eigen::Vector3d> line_points;
    std::vector<Eigen::Vector2i> lines;
    std::vector<Eigen::Vector3d> colors;

    std::shared_ptr<open3d::geometry::LineSet> ls_ptr(new open3d::geometry::LineSet());
    for (int i = 0; i < polygon.shell.size(); ++i)
    {
        auto idx = polygon.shell[i];
        Eigen::Vector3d line_point(points(idx, 0), points(idx, 1), points(idx, 2));
        line_points.push_back(line_point);
        int cur_point = static_cast<int>(line_points.size() - 1);
        colors.push_back(shell_color);
        if (i != polygon.shell.size() - 1)
        {
            lines.push_back({cur_point, cur_point + 1});
            colors.push_back(shell_color);
        }
    }
    for (auto& hole : polygon.holes)
    {
        for (int i = 0; i < hole.size(); ++i)
        {
            auto idx = hole[i];
            Eigen::Vector3d line_point(points(idx, 0), points(idx, 1), points(idx, 2));
            line_points.push_back(line_point);
            int cur_point = static_cast<int>(line_points.size() - 1);
            colors.push_back(hole_color);
            if (i != hole.size() - 1)
            {
                lines.push_back({cur_point, cur_point + 1});
                colors.push_back(hole_color);
            }
        }
    }
    ls_ptr->points_ = line_points;
    ls_ptr->lines_ = lines;
    ls_ptr->colors_ = colors;
    return ls_ptr;
}

} // namespace VisUtility