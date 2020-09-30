#include "Polylidar/Mesh/MeshHelper.hpp"
#include <iostream>
namespace Polylidar {

namespace MeshHelper {
constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();
const static double PL_NAN = std::numeric_limits<double>::quiet_NaN();

// Half Edge Constructors
HalfEdgeTriangulation::HalfEdgeTriangulation() : vertices(), triangles(), halfedges(), triangle_normals(), vertex_classes(), counter_clock_wise(true) {}

HalfEdgeTriangulation::HalfEdgeTriangulation(const Matrix<double>& in_vertices)
    : vertices(in_vertices), triangles(), halfedges(), triangle_normals(), vertex_classes(), counter_clock_wise(true)
{
}

HalfEdgeTriangulation::HalfEdgeTriangulation(Matrix<double>&& in_vertices)
    : vertices(std::move(in_vertices)), triangles(), halfedges(), triangle_normals(), vertex_classes(), counter_clock_wise(true)
{
}

HalfEdgeTriangulation::HalfEdgeTriangulation(Matrix<double>&& in_vertices, Matrix<size_t>&& in_triangles)
    : vertices(std::move(in_vertices)), triangles(std::move(in_triangles)), halfedges(), triangle_normals(), vertex_classes(), counter_clock_wise(true)
{
    ExtractHalfEdgesMatrix(triangles, halfedges);
}

HalfEdgeTriangulation::HalfEdgeTriangulation(Matrix<double>&& in_vertices, Matrix<size_t>&& in_triangles,
                                             Matrix<size_t>&& in_halfedges)
    : vertices(std::move(in_vertices)),
      triangles(std::move(in_triangles)),
      halfedges(std::move(in_halfedges)),
      triangle_normals(),
      vertex_classes(),
      counter_clock_wise(true)
{
}
// TriMesh::TriMesh() : HalfEdgeTriangulation(), triangle_normals() {}

// TriMesh::TriMesh(Matrix<double>&& in_vertices, Matrix<size_t>&& in_triangles, Matrix<size_t>&& in_halfedges)
//     : HalfEdgeTriangulation(std::move(in_vertices), std::move(in_triangles), std::move(in_halfedges)),
//       triangle_normals()
// {
// }

HalfEdgeTriangulation::HalfEdgeTriangulation(Matrix<double>&& in_vertices, Matrix<size_t>&& in_triangles,
                                             Matrix<size_t>&& in_halfedges, Matrix<double>&& in_triangle_normals)
    : vertices(std::move(in_vertices)),
      triangles(std::move(in_triangles)),
      halfedges(std::move(in_halfedges)),
      triangle_normals(std::move(in_triangle_normals)),
      vertex_classes(),
      counter_clock_wise(true)
{
}

void HalfEdgeTriangulation::SetTriangleNormals(const Matrix<double> &in_triangle_normals)
{
    triangle_normals = in_triangle_normals;
}

void HalfEdgeTriangulation::SetVertexClasses(const Matrix<uint8_t> &in_vertex_classes, bool copy)
{
    // if in_vertices_classes does NOT own the data then vertex_classes will also NOT own the data.
    // However, copy=True indicates that the data should be copied into vertex_classes and wil own the data
    if (copy && !in_vertex_classes.own_data)
    {
        vertex_classes = Matrix<uint8_t>::CopyFromDifferentType(in_vertex_classes.ptr, in_vertex_classes.rows, in_vertex_classes.cols);
    }
    else
    {
        vertex_classes = in_vertex_classes;
    }
    
}

void ComputeTriangleNormalsFromMatrix(const Matrix<double>& vertices, const Matrix<size_t>& triangles,
                                      Matrix<double>& triangle_normals_mat, const bool flip_normals)
{
    auto& num_triangles = triangles.rows;
    auto& triangle_normals = triangle_normals_mat.data;
    triangle_normals.resize(num_triangles * 3);
    for (size_t i = 0; i < num_triangles; ++i)
    {
        auto& pi0 = triangles(i, 0);
        auto& pi1 = triangles(i, 1);
        auto& pi2 = triangles(i, 2);

        std::array<double, 3> vv1 = {vertices(pi0, 0), vertices(pi0, 1), vertices(pi0, 2)};
        std::array<double, 3> vv2 = {vertices(pi1, 0), vertices(pi1, 1), vertices(pi1, 2)};
        std::array<double, 3> vv3 = {vertices(pi2, 0), vertices(pi2, 1), vertices(pi2, 2)};

        // two lines of triangle
        // V1 is starting index
        std::array<double, 3> u{{vv2[0] - vv1[0], vv2[1] - vv1[1], vv2[2] - vv1[2]}};
        std::array<double, 3> v{{vv3[0] - vv1[0], vv3[1] - vv1[1], vv3[2] - vv1[2]}};

        // cross product
        if (flip_normals)
        {
            Utility::Math::CrossProduct3(v, u, &triangle_normals[i * 3]);
        }
        else
        {
            Utility::Math::CrossProduct3(u, v, &triangle_normals[i * 3]);
        }
        
        // normalize
        Utility::Math::Normalize3(&triangle_normals[i * 3]);
    }

    // Ensure that the matrix ptr, row, col is updated
    // TODO - This kind of sucks because if you forget to do this there will be serious issues
    triangle_normals_mat.rows = num_triangles;
    triangle_normals_mat.cols = 3UL;
    triangle_normals_mat.ptr = triangle_normals.data();
}

void HalfEdgeTriangulation::ComputeTriangleNormals()
{
    ComputeTriangleNormalsFromMatrix(vertices, triangles, triangle_normals, !counter_clock_wise);
}

void ComputeTriangleNormals(const Matrix<double>& vertices, const std::vector<size_t>& triangles,
                            std::vector<double>& triangle_normals, const bool flip_normals)
{
    size_t num_triangles = static_cast<size_t>(triangles.size() / 3);
    triangle_normals.resize(num_triangles * 3);

    for (size_t i = 0; i < triangles.size(); i += 3)
    {
        auto& pi0 = triangles[i];
        auto& pi1 = triangles[i + 1];
        auto& pi2 = triangles[i + 2];

        std::array<double, 3> vv1 = {vertices(pi0, 0), vertices(pi0, 1), vertices(pi0, 2)};
        std::array<double, 3> vv2 = {vertices(pi1, 0), vertices(pi1, 1), vertices(pi1, 2)};
        std::array<double, 3> vv3 = {vertices(pi2, 0), vertices(pi2, 1), vertices(pi2, 2)};

        // two lines of triangle
        // V1 is starting index
        std::array<double, 3> u{{vv2[0] - vv1[0], vv2[1] - vv1[1], vv2[2] - vv1[2]}};
        std::array<double, 3> v{{vv3[0] - vv1[0], vv3[1] - vv1[1], vv3[2] - vv1[2]}};

        // cross product
        if (flip_normals)
        {
            Utility::Math::CrossProduct3(v, u, &triangle_normals[i * 3]);
        }
        else
        {
            Utility::Math::CrossProduct3(u, v, &triangle_normals[i * 3]);
        }
        // normalize
        Utility::Math::Normalize3(&triangle_normals[i]);
    }
}

HalfEdgeTriangulation CreateTriMeshFromVectors(std::vector<double>&& vertices, std::vector<size_t>&& triangles,
                                               std::vector<size_t>&& halfedges)
{
    size_t num_vertices = static_cast<size_t>(vertices.size() / 3);
    size_t num_triangles = static_cast<size_t>(triangles.size() / 3);
    size_t num_halfedges = static_cast<size_t>(halfedges.size() / 3);

    Matrix<double> vertices_matrix{std::move(vertices), num_vertices, 3};
    Matrix<size_t> triangles_matrix{std::move(triangles), num_triangles, 3};
    Matrix<size_t> halfedges_matrix{std::move(halfedges), num_halfedges, 3};

    return HalfEdgeTriangulation(std::move(vertices_matrix), std::move(triangles_matrix), std::move(halfedges_matrix));
}

HalfEdgeTriangulation CreateTriMeshFromVectors(Matrix<double>&& vertices, std::vector<size_t>&& triangles,
                                               std::vector<size_t>&& halfedges)
{
    size_t num_triangles = static_cast<size_t>(triangles.size() / 3);
    size_t num_halfedges = static_cast<size_t>(halfedges.size() / 3);

    Matrix<size_t> triangles_matrix{std::move(triangles), num_triangles, 3};
    Matrix<size_t> halfedges_matrix{std::move(halfedges), num_halfedges, 3};

    return HalfEdgeTriangulation(std::move(vertices), std::move(triangles_matrix), std::move(halfedges_matrix));
}

inline void deproject_points(const size_t i, const size_t j, float depth, const Matrix<double>& intrinsics,
                             const Matrix<double>& extrinsics, double& x, double& y, double& z)
{
    double z1 = static_cast<double>(depth);
    double x1 = (static_cast<double>(j) - intrinsics(0, 2)) * z1 / intrinsics(0, 0);
    double y1 = (static_cast<double>(i) - intrinsics(1, 2)) * z1 / intrinsics(1, 1);
    // Rotate
    x = extrinsics(0, 0) * x1 + extrinsics(0, 1) * y1 + extrinsics(0, 2) * z1 + extrinsics(0, 3);
    y = extrinsics(1, 0) * x1 + extrinsics(1, 1) * y1 + extrinsics(1, 2) * z1 + extrinsics(1, 3);
    z = extrinsics(2, 0) * x1 + extrinsics(2, 1) * y1 + extrinsics(2, 2) * z1 + extrinsics(2, 3);
}

std::vector<double> ExtractPointCloudFromFloatDepth(const Matrix<float>& im, const Matrix<double>& intrinsics,
                                                    const Matrix<double>& extrinsics, const size_t stride)
{
    std::vector<double> points;
    int stride_ = static_cast<int>(stride);
    int rows = static_cast<int>(im.rows);
    int cols = static_cast<int>(im.cols);
    int cols_stride = (cols + stride_ - 1) / stride_;
    int rows_stride = (rows + stride_ - 1) / stride_;
    points.resize(cols_stride * rows_stride * 3, PL_NAN);
#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), PL_OMP_MAX_THREAD_DEPTH_TO_PC);
    num_threads = std::max(num_threads, 1);
#pragma omp parallel for schedule(static) num_threads(num_threads)
#endif
    for (int i = 0; i < rows; i += stride_)
    {
        for (int j = 0; j < cols; j += stride_)
        {
            size_t p_idx = static_cast<size_t>((cols_stride * i / stride + j / stride) * 3);
            if (im(i, j) > 0)
                deproject_points(i, j, im(i, j), intrinsics, extrinsics, points[p_idx], points[p_idx + 1],
                                 points[p_idx + 2]);
        }
    }
    return points;
}

std::vector<size_t> ExtractHalfEdgesFromUniformMesh(size_t rows, size_t cols, std::vector<size_t>& triangles,
                                                    std::vector<size_t>& valid_tri, size_t stride)
{
    // constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();
    std::vector<size_t> halfedges(triangles.size(), INVALID_INDEX);
    // This represents the number of rows and columns of the downsampled POINT CLOUD
    size_t cols_stride = (cols + stride - 1) / stride;
    size_t rows_stride = (rows + stride - 1) / stride;
    // This represent the number of rows and columns of the UNIFORM TRIANGULAR MESH
    size_t cols_tris = cols_stride - 1;
    size_t rows_tris = rows_stride - 1;
    // #pragma omp parallel for schedule(static) // No speedup, tried many settings
    for (size_t i = 0; i < rows_tris; i++)
    {
        for (size_t j = 0; j < cols_tris; j++)
        {
            // These are the triangle indexes in the global full mesh
            size_t t_global_idx_first = (cols_tris * i + j) * 2;
            size_t t_global_idx_second = (cols_tris * i + j) * 2 + 1;
            // We convert these global meshes to our valid mesh indices
            auto t_valid_idx_first = valid_tri[t_global_idx_first];
            auto t_valid_idx_second = valid_tri[t_global_idx_second];
            // Valid triangles indices bordering the first triangle
            size_t t_valid_idx_top = 0;
            size_t t_valid_idx_right = 0;
            // valid triangle indices bordering the second triangle
            size_t t_valid_idx_bottom = 0;
            size_t t_valid_idx_left = 0;
            // Check if first triangle is valid, if invalid its not in our mesh
            if (t_valid_idx_first != INVALID_INDEX)
            {
                // Check if we are on the top of the depth image
                if (i == 0)
                {
                    t_valid_idx_top = INVALID_INDEX; // indicates this edge has no border
                }
                else
                {
                    // gets the triangle one row up from this one, math is from implicit structure
                    size_t t_global_idx_top = t_global_idx_first - 2 * cols_tris + 1;
                    // convert to valid mesh index
                    t_valid_idx_top = valid_tri[t_global_idx_top];
                }
                // check if we are on the right side of the depth Image
                if (j >= cols_tris - 1)
                {
                    t_valid_idx_right = INVALID_INDEX; // indicates this edge has no border
                }
                else
                {
                    // gets the triangle one cell to the right, math is from implicit structure
                    size_t t_global_idx_right = t_global_idx_first + 3;
                    t_valid_idx_right = valid_tri[t_global_idx_right];
                }
                // Set the edges if they are valid
                // if (t_valid_idx_top != INVALID_INDEX) halfedges[size_t(t_valid_idx_first * 3)] = t_valid_idx_top * 3;
                // if (t_valid_idx_right != INVALID_INDEX)
                //     halfedges[size_t(t_valid_idx_first * 3 + 1)] = t_valid_idx_right * 3 + 1;
                // if (t_valid_idx_second != INVALID_INDEX)
                //     halfedges[size_t(t_valid_idx_first * 3 + 2)] = t_valid_idx_second * 3 + 2;

                if (t_valid_idx_top != INVALID_INDEX) halfedges[size_t(t_valid_idx_first * 3 + 1)] = t_valid_idx_top * 3 + 1;
                if (t_valid_idx_right != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_first * 3)] = t_valid_idx_right * 3;
                if (t_valid_idx_second != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_first * 3 + 2)] = t_valid_idx_second * 3 + 2;
            }
            // Check if second triangle is valid, if invalid its not in our mesh
            if (t_valid_idx_second != INVALID_INDEX)
            {
                // We have a valid second triangle
                // Check if we are on the bottom of the depth image
                if (i == rows_tris - 1)
                {
                    t_valid_idx_bottom = INVALID_INDEX;
                }
                else
                {
                    size_t t_global_idx_bottom = t_global_idx_second + 2 * cols_tris - 1;
                    t_valid_idx_bottom = valid_tri[t_global_idx_bottom];
                }
                // Check if we are on the left side of the RGBD Image, if so than we have a border on the left
                if (j == 0)
                {
                    t_valid_idx_left = INVALID_INDEX;
                }
                else
                {
                    size_t t_global_idx_left = t_global_idx_second - 3;
                    t_valid_idx_left = valid_tri[t_global_idx_left];
                }
                // Set Edges
                // if (t_valid_idx_bottom != INVALID_INDEX)
                //     halfedges[size_t(t_valid_idx_second * 3)] = t_valid_idx_bottom * 3;
                // if (t_valid_idx_left != INVALID_INDEX)
                //     halfedges[size_t(t_valid_idx_second * 3 + 1)] = t_valid_idx_left * 3 + 1;
                // if (t_valid_idx_first != INVALID_INDEX)
                //     halfedges[size_t(t_valid_idx_second * 3 + 2)] = t_valid_idx_first * 3 + 2;
                if (t_valid_idx_bottom != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_second * 3 + 1)] = t_valid_idx_bottom * 3 + 1;
                if (t_valid_idx_left != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_second * 3)] = t_valid_idx_left * 3;
                if (t_valid_idx_first != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_second * 3 + 2)] = t_valid_idx_first * 3 + 2;
            }
        }
    }
    return halfedges;
}

std::tuple<std::vector<size_t>, std::vector<size_t>>
CreateUniformMesh(const size_t rows, const size_t cols, const Matrix<double>& points_2D, const size_t stride)
{
    std::vector<size_t> triangles;
    // This represents the number of rows and columns of the downsampled POINT CLOUD
    // size_t cols_stride = static_cast<size_t>(ceil(cols / static_cast<float>(stride)));
    size_t cols_stride = (cols + stride - 1) / stride;
    size_t rows_stride = (rows + stride - 1) / stride;
    // This represent the number of rows and columns of the UNIFORM TRIANGULAR MESH
    size_t cols_tris = cols_stride - 1;
    size_t rows_tris = rows_stride - 1;
    // These are the maximum number of triangles that can ever be in the mesh
    size_t max_triangles = cols_tris * rows_tris * 2;
    // This will count valid points and triangles
    size_t tri_cnt = 0;
    size_t pix_cnt = 0;
    // Invalid Triangle Marker
    // constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();
    std::vector<size_t> valid_tri(max_triangles, INVALID_INDEX);
    // Reserve memory for triangles
    triangles.reserve(max_triangles);
    for (size_t i = 0; i < rows_tris; i++)
    {
        for (size_t j = 0; j < cols_tris; j++)
        {
            size_t p1_idx = i * cols_stride + j;
            size_t p2_idx = i * cols_stride + j + 1;
            size_t p3_idx = (i + 1) * cols_stride + j + 1;
            size_t p4_idx = (i + 1) * cols_stride + j;

            auto& p1 = points_2D(p1_idx, 2);
            auto& p2 = points_2D(p2_idx, 2);
            auto& p3 = points_2D(p3_idx, 2);
            auto& p4 = points_2D(p4_idx, 2);

            if (!std::isnan(p1) && !std::isnan(p2) && !std::isnan(p3))
            {
                triangles.push_back(p3_idx);
                triangles.push_back(p2_idx);
                triangles.push_back(p1_idx);
                // triangles.push_back(p1_idx);
                // triangles.push_back(p2_idx);
                // triangles.push_back(p3_idx);
                valid_tri[pix_cnt * 2] = tri_cnt;
                tri_cnt++;
            }
            if (!std::isnan(p3) && !std::isnan(p4) && !std::isnan(p1))
            {
                triangles.push_back(p1_idx);
                triangles.push_back(p4_idx);
                triangles.push_back(p3_idx);
                // triangles.push_back(p3_idx);
                // triangles.push_back(p4_idx);
                // triangles.push_back(p1_idx);
                valid_tri[pix_cnt * 2 + 1] = tri_cnt;
                tri_cnt++;
            }
            pix_cnt++;
        }
    }
    return std::make_tuple(std::move(triangles), std::move(valid_tri));
}

std::tuple<std::vector<double>, std::vector<size_t>, std::vector<size_t>>
ExtractUniformMeshFromFloatDepth(const Matrix<float>& im, const Matrix<double>& intrinsics,
                                 const Matrix<double>& extrinsics, const size_t stride)
{
    std::vector<size_t> triangles;
    std::vector<size_t> valid_tri;
    // auto t0 = std::chrono::high_resolution_clock::now();
    std::vector<double> points = ExtractPointCloudFromFloatDepth(im, intrinsics, extrinsics, stride);
    Matrix<double> points_2D(points.data(), points.size() / 3, 3);
    // auto t1 = std::chrono::high_resolution_clock::now();
    // float elapsed_d = static_cast<std::chrono::duration<float, std::milli>>(t1 - t0).count();
    // std::cout << "Point Cloud Extraction took " << elapsed_d << " milliseconds" << std::endl;
    std::tie(triangles, valid_tri) = CreateUniformMesh(im.rows, im.cols, points_2D, stride);
    // auto t2 = std::chrono::high_resolution_clock::now();
    // elapsed_d = static_cast<std::chrono::duration<float, std::milli>>(t2 - t1).count();
    // std::cout << "Create Uniform Mesh took " << elapsed_d << " milliseconds" << std::endl;
    std::vector<size_t> halfedges = ExtractHalfEdgesFromUniformMesh(im.rows, im.cols, triangles, valid_tri, stride);
    // auto t3 = std::chrono::high_resolution_clock::now();
    // elapsed_d = static_cast<std::chrono::duration<float, std::milli>>(t3 - t2).count();
    // std::cout << "Extract Half Edge took " << elapsed_d << " milliseconds" << std::endl;
    // std::cout << "extractUniformMeshFromFloatDepth C++ : " << points[0] << " Address:" <<  &points[0] << std::endl;
    return std::make_tuple(std::move(points), std::move(triangles), std::move(halfedges));
}

HalfEdgeTriangulation ExtractTriMeshFromFloatDepth(const Matrix<float>& im, const Matrix<double>& intrinsics,
                                                   const Matrix<double>& extrinsics, const size_t stride,
                                                   const bool calc_normals)
{
    std::vector<double> vertices;
    std::vector<size_t> triangles;
    std::vector<size_t> halfedges;
    std::tie(vertices, triangles, halfedges) = ExtractUniformMeshFromFloatDepth(im, intrinsics, extrinsics, stride);
    auto triangulation = CreateTriMeshFromVectors(std::move(vertices), std::move(triangles), std::move(halfedges));
    if (calc_normals)
    {
        triangulation.ComputeTriangleNormals();
    }
    return triangulation;
}

std::tuple<HalfEdgeTriangulation, VUI> ExtractTriMeshFromOrganizedPointCloud(Matrix<double> &points_2D, const size_t rows,
                                                            const size_t cols, const size_t stride,
                                                            const bool calc_normals)
{
    std::vector<size_t> triangles;
    std::vector<size_t> valid_tri;
    std::tie(triangles, valid_tri) = CreateUniformMesh(rows, cols, points_2D, stride);
    std::vector<size_t> halfedges = ExtractHalfEdgesFromUniformMesh(rows, cols, triangles, valid_tri, stride);
    auto triangulation = CreateTriMeshFromVectors(std::move(points_2D), std::move(triangles), std::move(halfedges));
    if (calc_normals)
    {
        triangulation.ComputeTriangleNormals();
    }
    return std::make_tuple(std::move(triangulation), std::move(valid_tri));
}

HalfEdgeTriangulation CreateTriMeshCopy(Matrix<double>& vertices, Matrix<int>& triangles, const bool calc_normals)
{
    auto new_triangles = Matrix<size_t>::CopyFromDifferentType<int>(triangles.ptr, triangles.rows, triangles.cols);
    auto new_vertices = Matrix<double>(vertices);

    HalfEdgeTriangulation mesh(std::move(new_vertices), std::move(new_triangles));
    if (calc_normals)
    {
        mesh.ComputeTriangleNormals();
    }
    return mesh;
}

} // namespace MeshHelper
} // namespace Polylidar
