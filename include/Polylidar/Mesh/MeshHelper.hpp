#ifndef MESHHELPER
#define MESHHELPER
#include <vector>
#include <array>

#include <unordered_map>
#include <parallel_hashmap/phmap.h>

#include "Polylidar/Types.hpp"

#if defined(_OPENMP)
#include <omp.h>
#define PL_OMP_MAX_THREAD_DEPTH_TO_PC 8
#endif

#define DEFAULT_CALC_NORMALS true

namespace Polylidar {

namespace MeshHelper {

#ifdef PL_USE_STD_UNORDERED_MAP
template <typename T, typename G>
using unordered_map = std::unordered_map<T, G>;
#else
template <typename T, typename G>
using unordered_map = phmap::flat_hash_map<T, G>;
#endif

using MatX2I = std::vector<std::array<size_t, 2>>;

class HalfEdgeTriangulation
{

  public:
    Matrix<double> vertices;
    Matrix<size_t> triangles;
    Matrix<size_t> halfedges;
    Matrix<double> triangle_normals;

    HalfEdgeTriangulation();
    HalfEdgeTriangulation(HalfEdgeTriangulation&& other) = default;
    HalfEdgeTriangulation(const Matrix<double> &in_vertices);
    HalfEdgeTriangulation(Matrix<double>&& in_vertices);
    HalfEdgeTriangulation(Matrix<double>&& in_vertices, Matrix<size_t>&& in_triangles, Matrix<size_t>&& in_halfedges);
    HalfEdgeTriangulation(Matrix<double>&& in_vertices, Matrix<size_t>&& in_triangles, Matrix<size_t>&& in_halfedges, Matrix<double> &&in_triangle_normals);
    void ComputeTriangleNormals();
  private:
};

// class TriMesh : public HalfEdgeTriangulation
// {
//   public:
//     Matrix<double> triangle_normals;
//     TriMesh();
//     TriMesh(Matrix<double>&& in_vertices, Matrix<size_t>&& in_triangles, Matrix<size_t>&& in_halfedges);
//     TriMesh(Matrix<double>&& in_vertices, Matrix<size_t>&& in_triangles, Matrix<size_t>&& in_halfedges,
//             Matrix<double> in_triangle_normals);
//     void ComputeTriangleNormals();
// };

inline size_t CantorMapping(const size_t k1, const size_t k2)
{
    auto dk1 = static_cast<double>(k1);
    auto dk2 = static_cast<double>(k2);
    auto mapping = static_cast<size_t>(((dk1 + dk2) * (dk1 + dk2 + 1)) / 2.0 + dk2);
    return mapping;
}

inline std::vector<size_t> ExtractHalfEdges(const std::vector<size_t>& triangles)
{
    // auto before = std::chrono::high_resolution_clock::now();
    size_t max_limit = std::numeric_limits<size_t>::max();

    std::vector<size_t> halfedges(triangles.size(), max_limit);
    MatX2I halfedges_pi(triangles.size());
    unordered_map<size_t, size_t> vertex_indices_to_half_edge_index;
    vertex_indices_to_half_edge_index.reserve(triangles.size());

    for (size_t triangle_index = 0; triangle_index < triangles.size(); triangle_index += 3)
    {
        const size_t* triangle = &triangles[triangle_index];
        size_t& num_half_edges = triangle_index;

        size_t he_0_index = num_half_edges;
        size_t he_1_index = num_half_edges + 1;
        size_t he_2_index = num_half_edges + 2;
        size_t he_0_mapped = CantorMapping(triangle[0], triangle[1]);
        size_t he_1_mapped = CantorMapping(triangle[1], triangle[2]);
        size_t he_2_mapped = CantorMapping(triangle[2], triangle[0]);

        std::array<size_t, 2>& he_0 = halfedges_pi[he_0_index];
        std::array<size_t, 2>& he_1 = halfedges_pi[he_1_index];
        std::array<size_t, 2>& he_2 = halfedges_pi[he_2_index];

        he_0[0] = triangle[0];
        he_0[1] = triangle[1];
        he_1[0] = triangle[1];
        he_1[1] = triangle[2];
        he_2[0] = triangle[2];
        he_2[1] = triangle[0];

        // Check for Non Manifold Mesh?
        vertex_indices_to_half_edge_index[he_0_mapped] = he_0_index;
        vertex_indices_to_half_edge_index[he_1_mapped] = he_1_index;
        vertex_indices_to_half_edge_index[he_2_mapped] = he_2_index;
    }

    for (size_t this_he_index = 0; this_he_index < halfedges.size(); this_he_index++)
    {
        size_t& that_he_index = halfedges[this_he_index];
        std::array<size_t, 2>& this_he = halfedges_pi[this_he_index];
        size_t that_he_mapped = CantorMapping(this_he[1], this_he[0]);
        if (that_he_index == max_limit &&
            vertex_indices_to_half_edge_index.find(that_he_mapped) != vertex_indices_to_half_edge_index.end())
        {
            size_t twin_he_index = vertex_indices_to_half_edge_index[that_he_mapped];
            halfedges[this_he_index] = twin_he_index;
            halfedges[twin_he_index] = this_he_index;
        }
    }
    return halfedges;
}

inline void crossProduct3(const std::array<double, 3>& u, const std::array<double, 3>& v, double* normal)
{
    // cross product
    normal[0] = u[1] * v[2] - u[2] * v[1];
    normal[1] = u[2] * v[0] - u[0] * v[2];
    normal[2] = u[0] * v[1] - u[1] * v[0];
}

inline void normalize3(double* normal)
{
    auto norm = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
    normal[0] /= norm;
    normal[1] /= norm;
    normal[2] /= norm;
}

void ComputeTriangleNormalsFromMatrix(const Matrix<double>& vertices, const Matrix<size_t>& triangles,
                                      Matrix<double>& triangle_normals_mat);
void ComputeTriangleNormals(const Matrix<double>& vertices, const std::vector<size_t>& triangles,
                            std::vector<double>& triangle_normals);

HalfEdgeTriangulation CreateTriMeshFromVectors(std::vector<double>&& vertices, std::vector<size_t>&& triangles,
                                 std::vector<size_t>&& halfedges);
// TriMesh CreateTriMeshCopy(const double* vertices_ptr, size_t num_vertices, const size_t* triangles_ptr,
//                           size_t num_triangles);
// TriMesh CreateTriMeshCopy(const double* vertices_ptr, size_t num_vertices, const int* triangles_ptr,
//                           size_t num_triangles);
std::vector<double> ExtractPointCloudFromFloatDepth(const Matrix<float>& im, const Matrix<double>& intrinsics,
                                                    const Matrix<double>& extrinsics, const size_t stride);
std::tuple<std::vector<double>, std::vector<size_t>, std::vector<size_t>>
ExtractUniformMeshFromFloatDepth(const Matrix<float>& im, const Matrix<double>& intrinsics,
                                 const Matrix<double>& extrinsics, const size_t stride);
HalfEdgeTriangulation ExtractTriMeshFromFloatDepth(const Matrix<float>& im, const Matrix<double>& intrinsics,
                                                 const Matrix<double>& extrinsics, const size_t stride,
                                                 const bool calc_normals = DEFAULT_CALC_NORMALS);
HalfEdgeTriangulation ExtractTriMeshFromOrganizedPointCloud(Matrix<double> points_2D, const size_t rows,
                                                          const size_t cols, const size_t stride,
                                                          const bool calc_normals);

} // namespace MeshHelper
} // namespace Polylidar

#endif