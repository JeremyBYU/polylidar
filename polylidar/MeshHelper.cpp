#include "polylidar/Mesh/MeshHelper.hpp"
namespace polylidar
{

namespace MeshHelper
{
const static double PL_NAN = std::numeric_limits<double>::quiet_NaN();

    // Half Edge Constructors
HalfEdgeTriangulation::HalfEdgeTriangulation()
    : coords(),
      triangles(),
      halfedges() {}

HalfEdgeTriangulation::HalfEdgeTriangulation(polylidar::Matrix<double> &in_coords)
    : coords(in_coords),
      triangles(),
      halfedges() {}

#ifdef PY_EXTENSION
HalfEdgeTriangulation::HalfEdgeTriangulation(pybind11::array_t<double> nparray)
    : coords(),
      triangles(),
      halfedges()
{
    auto info = nparray.request();
    std::vector<size_t> shape({(size_t)info.shape[0], (size_t)info.shape[1]});
    coords.ptr = (double *)info.ptr;
    coords.rows = shape[0];
    coords.cols = shape[1];
}
HalfEdgeTriangulation::HalfEdgeTriangulation(polylidar::Matrix<double> &in_coords, pybind11::array_t<size_t> triangles_, pybind11::array_t<size_t> halfedges_)
    : coords(in_coords),
      triangles(),
      halfedges()
{
    // Copy to vector
    // allocate std::vector (to pass to the C++ function)
    triangles.resize(triangles_.size());
    std::memcpy(triangles.data(), triangles_.data(), triangles_.size() * sizeof(size_t));

    // allocate std::vector (to pass to the C++ function)
    halfedges.resize(halfedges_.size());
    std::memcpy(halfedges.data(), halfedges_.data(), halfedges_.size() * sizeof(size_t));
}
#endif

// This class is a specialization of HalfEdgeTriangulation
// This class will actually own the memory of the vertices and will always be Three Dimensional
// This constructor is destructive towards the input data, meaning the data is now owned by the TriMesh
TriMesh::TriMesh(std::vector<double> &in_vertices, std::vector<size_t> &in_triangles, std::vector<size_t> &in_halfedges)
    : HalfEdgeTriangulation(),
      vertices(),
      triangle_normals()
{
    vertices.swap(in_vertices);
    triangles.swap(in_triangles);
    halfedges.swap(in_halfedges);
    // Point the 2D coords Matrix to vertices data;
    coords.cols = 3;
    coords.rows = vertices.size() / 3;
    coords.ptr = vertices.data();
}

TriMesh::TriMesh(std::vector<double> &in_vertices, std::vector<size_t> &in_triangles)
    : HalfEdgeTriangulation(),
      vertices(),
      triangle_normals()
{
    vertices.swap(in_vertices);
    triangles.swap(in_triangles);
    auto halfedges_ = MeshHelper::ExtractHalfEdges(triangles);
    halfedges.swap(halfedges_);
    // Point the 2D coords Matrix to vertices data;
    coords.cols = 3;
    coords.rows = vertices.size() / 3;
    coords.ptr = vertices.data();
}

TriMesh::TriMesh(const polylidar::Matrix<double> &in_coords, std::vector<size_t> &in_triangles, std::vector<size_t> &in_halfedges, const bool copy_vertices)
    : HalfEdgeTriangulation(),
      vertices(),
      triangle_normals()
{
    // TODO COPY COORDS TO VERTICES IF REQUESTED
    // vertices.swap(in_vertices);
    triangles.swap(in_triangles);
    halfedges.swap(in_halfedges);
    // Point the 2D coords Matrix to vertices data;
    coords = in_coords;
}

TriMesh::TriMesh()
    : HalfEdgeTriangulation(),
      vertices(),
      triangle_normals()
{
}

void ComputeTriangleNormals(const Matrix<double> &vertices, const std::vector<size_t> &triangles, std::vector<double> &triangle_normals)
{
    size_t num_triangles = static_cast<size_t>(triangles.size() / 3);
    triangle_normals.resize(num_triangles * 3);

    for (size_t i = 0; i < triangles.size(); i += 3)
    {
        auto &pi0 = triangles[i];
        auto &pi1 = triangles[i + 1];
        auto &pi2 = triangles[i + 2];

        std::array<double, 3> vv1 = {vertices(pi0, 0), vertices(pi0, 1), vertices(pi0, 2)};
        std::array<double, 3> vv2 = {vertices(pi1, 0), vertices(pi1, 1), vertices(pi1, 2)};
        std::array<double, 3> vv3 = {vertices(pi2, 0), vertices(pi2, 1), vertices(pi2, 2)};

        // two lines of triangle
        // V1 is starting index
        std::array<double, 3> u{{vv2[0] - vv1[0], vv2[1] - vv1[1], vv2[2] - vv1[2]}};
        std::array<double, 3> v{{vv3[0] - vv1[0], vv3[1] - vv1[1], vv3[2] - vv1[2]}};

        // cross product
        crossProduct3(v, u, &triangle_normals[i]);
        // normalize
        normalize3(&triangle_normals[i]);
    }
}

TriMesh CreateTriMeshCopy(const double *vertices_ptr, size_t num_vertices, const size_t *triangles_ptr, size_t num_triangles)
{
    auto vertices_elements = num_vertices * 3;
    auto triangles_elements = num_triangles * 3;
    std::vector<double> vec_vertices(vertices_ptr, vertices_ptr + vertices_elements);
    std::vector<size_t> vec_triangles(triangles_ptr, triangles_ptr + triangles_elements);
    TriMesh tri_mesh(vec_vertices, vec_triangles);
    ComputeTriangleNormals(tri_mesh.coords, tri_mesh.triangles, tri_mesh.triangle_normals);

    return tri_mesh;
}
// Open 3D uses integers instead of size_t
TriMesh CreateTriMeshCopy(const double *vertices_ptr, size_t num_vertices, const int *triangles_ptr, size_t num_triangles)
{
    auto vertices_elements = num_vertices * 3;
    auto triangles_elements = num_triangles * 3;
    std::vector<double> vec_vertices(vertices_ptr, vertices_ptr + vertices_elements);
    std::vector<size_t> vec_triangles(triangles_elements);
    // Copy int to size_t
    for(size_t i = 0; i < triangles_elements; i++)
    {
        vec_triangles[i] = static_cast<size_t>(triangles_ptr[i]);
    }

    TriMesh tri_mesh(vec_vertices, vec_triangles);
    ComputeTriangleNormals(tri_mesh.coords, tri_mesh.triangles, tri_mesh.triangle_normals);

    return tri_mesh;
}


inline void deproject_points(const size_t i, const size_t j, float depth, const Matrix<double> &intrinsics, const Matrix<double> &extrinsics, double &x, double &y, double &z)
{
    double z1 = static_cast<double>(depth);
    double x1 = (static_cast<double>(j) - intrinsics(0, 2)) * z1 / intrinsics(0, 0);
    double y1 = (static_cast<double>(i) - intrinsics(1, 2)) * z1 / intrinsics(1, 1);
    // Rotate
    x = extrinsics(0, 0) * x1 + extrinsics(0, 1) * y1 + extrinsics(0, 2) * z1 + extrinsics(0, 3);
    y = extrinsics(1, 0) * x1 + extrinsics(1, 1) * y1 + extrinsics(1, 2) * z1 + extrinsics(1, 3);
    z = extrinsics(2, 0) * x1 + extrinsics(2, 1) * y1 + extrinsics(2, 2) * z1 + extrinsics(2, 3);
}

std::vector<double> ExtractPointCloudFromFloatDepth(const Matrix<float> &im, const Matrix<double> &intrinsics, const Matrix<double> &extrinsics, const size_t stride)
{
    std::vector<double> points;
    auto rows = im.rows;
    auto cols = im.cols;
    size_t cols_stride = (cols + stride - 1) / stride;
    size_t rows_stride = (rows + stride - 1) / stride;
    points.resize(cols_stride * rows_stride * 3, PL_NAN);
#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), PL_OMP_MAX_THREAD_DEPTH_TO_PC);
#pragma omp parallel for schedule(static) num_threads(num_threads)
#endif
    for (size_t i = 0; i < rows; i += stride)
    {
        for (size_t j = 0; j < cols; j += stride)
        {
            size_t p_idx = static_cast<size_t>((cols_stride * i / stride + j / stride) * 3);
            if (im(i, j) > 0)
                deproject_points(i, j, im(i, j), intrinsics, extrinsics, points[p_idx], points[p_idx + 1], points[p_idx + 2]);
        }
    }
    // std::cout << "Point Count: " << pnt_cnt << "; Expected: "<< cols_stride * rows_stride <<std::endl;
    // std::cout << "extractPointCloudFromFloatDepth C++ : " << points[0] << " Address:" <<  &points[0] << std::endl;
    return points;
}

std::vector<size_t> ExtractHalfEdgesFromUniformMesh(size_t rows, size_t cols, std::vector<size_t> &triangles,
                                                    std::vector<size_t> &valid_tri, size_t stride)
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
        // int tid = omp_get_thread_num();
        // std::cout << "Hello from " << tid << std::endl;
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
                if (t_valid_idx_top != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_first * 3)] = t_valid_idx_top * 3;
                if (t_valid_idx_right != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_first * 3 + 1)] = t_valid_idx_right * 3 + 1;
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
                if (t_valid_idx_bottom != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_second * 3)] = t_valid_idx_bottom * 3;
                if (t_valid_idx_left != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_second * 3 + 1)] = t_valid_idx_left * 3 + 1;
                if (t_valid_idx_first != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_second * 3 + 2)] = t_valid_idx_first * 3 + 2;
            }
        }
    }
    return halfedges;
}

std::tuple<std::vector<size_t>, std::vector<size_t>> CreateUniformMesh(const size_t rows, const size_t cols, const Matrix<double> &points_2D, const size_t stride)
{
    std::vector<size_t> triangles;
    // This represents the number of rows and columns of the downsampled POINT CLOUD
    // size_t cols_stride = static_cast<size_t>(ceil(cols / static_cast<float>(stride)));
    size_t cols_stride = (cols + stride - 1) / stride;
    size_t rows_stride = (rows + stride - 1) / stride;
    // size_t rows_stride = static_cast<size_t>(ceil(rows / static_cast<float>(stride)));
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

            auto &p1 = points_2D(p1_idx, 2);
            auto &p2 = points_2D(p2_idx, 2);
            auto &p3 = points_2D(p3_idx, 2);
            auto &p4 = points_2D(p4_idx, 2);

            if (!std::isnan(p1) && !std::isnan(p2) && !std::isnan(p3))
            {
                triangles.push_back(p1_idx);
                triangles.push_back(p2_idx);
                triangles.push_back(p3_idx);
                valid_tri[pix_cnt * 2] = tri_cnt;
                tri_cnt++;
            }
            if (!std::isnan(p3) && !std::isnan(p4) && !std::isnan(p1))
            {
                triangles.push_back(p3_idx);
                triangles.push_back(p4_idx);
                triangles.push_back(p1_idx);
                valid_tri[pix_cnt * 2 + 1] = tri_cnt;
                tri_cnt++;
            }
            pix_cnt++;
        }
    }
    return std::make_tuple(std::move(triangles), std::move(valid_tri));
}

std::tuple<std::vector<double>, std::vector<size_t>, std::vector<size_t>> ExtractUniformMeshFromFloatDepth(const Matrix<float> &im, const Matrix<double> &intrinsics, const Matrix<double> &extrinsics, const size_t stride)
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

MeshHelper::TriMesh ExtractTriMeshFromFloatDepth(const Matrix<float> &im, const Matrix<double> &intrinsics, const Matrix<double> &extrinsics, const size_t stride, const bool calc_normals)
{
    std::vector<double> vertices;
    std::vector<size_t> triangles;
    std::vector<size_t> halfedges;
    std::tie(vertices, triangles, halfedges) = ExtractUniformMeshFromFloatDepth(im, intrinsics, extrinsics, stride);
    MeshHelper::TriMesh triangulation(vertices, triangles, halfedges);
    if (calc_normals)
    {
        MeshHelper::ComputeTriangleNormals(triangulation.coords, triangulation.triangles, triangulation.triangle_normals);
    }
    return triangulation;
}

MeshHelper::TriMesh ExtractTriMeshFromOrganizedPointCloud(const Matrix<double> points_2D, const size_t rows, const size_t cols, const size_t stride, const bool calc_normals)
{
    std::vector<size_t> triangles;
    std::vector<size_t> valid_tri;
    std::tie(triangles, valid_tri) = CreateUniformMesh(rows, cols, points_2D, stride);
    std::vector<size_t> halfedges = ExtractHalfEdgesFromUniformMesh(rows, cols, triangles, valid_tri, stride);

    MeshHelper::TriMesh triangulation(points_2D, triangles, halfedges, false);
    if (calc_normals)
    {
        MeshHelper::ComputeTriangleNormals(triangulation.coords, triangulation.triangles, triangulation.triangle_normals);
    }
    return triangulation;
}

} // namespace MeshHelper
} // namespace polylidar
