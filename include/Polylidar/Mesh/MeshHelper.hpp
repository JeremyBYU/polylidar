#ifndef MESHHELPER
#define MESHHELPER
#include <vector>
#include <array>

#include <unordered_map>
#include <parallel_hashmap/phmap.h>

#include "Polylidar/Types.hpp"
#include "Polylidar/UtilityMath.hpp"

#if defined(_OPENMP)
#include <omp.h>
#define PL_OMP_MAX_THREAD_DEPTH_TO_PC 8
#endif

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
/**
 * This class hold all the datastructures in our meshes (vertices, triangles, halfedges, etc.).
 * 
 */
class HalfEdgeTriangulation
{

  public:
    /** @brief Vertices in the mesh, N X 2 or N X 3 */
    Matrix<double> vertices;
    /** @brief Triangles in the mesh, K X 3 */
    Matrix<size_t> triangles;
    /** @brief Half-edge mapping in the mesh, K X 3. Every triangle has three oriented half-edges.
     * Each half-edge has unique id which is mapped to the 2D index of all triangles, e.g. the half-edges of triangle k
     * are [3*k, 3*k + 1, 3*k + 2]. Halfedges array provides the twin/opposite/shared half-edge id. e.g., The twin
     * half-edge of first edge for the triangle k is halfedges(k, 0)
     */
    Matrix<size_t> halfedges;
    /** @brief Triangle normals in the mesh (normalized), K X 3 */
    Matrix<double> triangle_normals;
    /** @brief Direction of travel for oriented half-edges around a triangle */
    bool counter_clock_wise;

    /**
     * @brief Construct an empty HalfEdgeTriangulation object
     *
     */
    HalfEdgeTriangulation();

    /**
     * @brief Copy an existing HalfEdgeTriangulation object
     *
     */
    HalfEdgeTriangulation& operator=(const HalfEdgeTriangulation& other) = default;
    /**
     * @brief Move (no copy) an existing HalfEdgeTriangulation object to another
     *
     * @param other
     */
    HalfEdgeTriangulation(HalfEdgeTriangulation&& other) = default;
    /**
     * @brief Create a new HalfEdgeTriangulation object with only vertices (copied). Will be triangulated later.
     *
     * @param in_vertices
     */
    HalfEdgeTriangulation(const Matrix<double>& in_vertices);
    /**
     * @brief Create a new HalfEdgeTriangulation object with only vertices (moved/no copy). Will be triangulated later.
     *
     * @param in_vertices
     */
    HalfEdgeTriangulation(Matrix<double>&& in_vertices);
    /**
     * @brief Create a new HalfEdgeTriangulation object with vertices and triangles (moved/no copy).
     *
     * @param in_vertices
     * @param in_triangles
     */
    HalfEdgeTriangulation(Matrix<double>&& in_vertices, Matrix<size_t>&& in_triangles);
    /**
     * @brief Create a new HalfEdgeTriangulation object with vertices, triangles, halfedges (moved/no copy).
     *
     * @param in_vertices
     * @param in_triangles
     * @param in_halfedges
     */
    HalfEdgeTriangulation(Matrix<double>&& in_vertices, Matrix<size_t>&& in_triangles, Matrix<size_t>&& in_halfedges);
    /**
     * @brief Create a new HalfEdgeTriangulation object with vertices, triangles, halfedges, and triangle normals
     * (moved/no copy).
     *
     * @param in_vertices
     * @param in_triangles
     * @param in_halfedges
     * @param in_triangle_normals
     */
    HalfEdgeTriangulation(Matrix<double>&& in_vertices, Matrix<size_t>&& in_triangles, Matrix<size_t>&& in_halfedges,
                          Matrix<double>&& in_triangle_normals);
    /**
     * @brief Set the Triangle Normals of the mesh
     *
     * @param in_triangle_normals
     */
    void SetTriangleNormals(const Matrix<double>& in_triangle_normals);
    /**
     * @brief Compute triangle normals using vertex and triangle datastructures
     *
     */
    void ComputeTriangleNormals();

  private:
};

inline size_t CantorMapping(const size_t k1, const size_t k2)
{
    auto dk1 = static_cast<double>(k1);
    auto dk2 = static_cast<double>(k2);
    auto mapping = static_cast<size_t>(((dk1 + dk2) * (dk1 + dk2 + 1)) / 2.0 + dk2);
    return mapping;
}

/**
 * @brief Extract the halfedges from a flattened triangles array datastructure.
 * `triangles` is simply the contiguous buffer of the the N X K triangles structure.
 * TODO: This should really be private/nested namespace
 *
 * @param triangles
 * @return std::vector<size_t>      This is the flattened contiguous buffer of halfedges
 */
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

/**
 * @brief Extracts the halfedges from a triangular mesh
 *
 * @param triangles             Input triangles
 * @param halfedges             Output halfedges into this paramter (mutated!)
 */
inline void ExtractHalfEdgesMatrix(const Matrix<size_t>& triangles, Matrix<size_t>& halfedges)
{
    auto& triangles_vec = triangles.data;
    // Extract half edges as a vector
    auto halfedges_vec = ExtractHalfEdges(triangles_vec);
    halfedges.data = std::move(halfedges_vec);
    // Update the ptr to the new data, set rows and columns
    halfedges.UpdatePtrFromData(triangles.rows, triangles.cols);
}

/**
 * @brief Compute Triangle Normals with Matrix Datastructures
 *
 * @param vertices
 * @param triangles
 * @param triangle_normals_mat          Results will be stored in here
 * @param flip_normals                  If true will flip the normals
 */
void ComputeTriangleNormalsFromMatrix(const Matrix<double>& vertices, const Matrix<size_t>& triangles,
                                      Matrix<double>& triangle_normals_mat, const bool flip_normals = false);

/**
 * @brief Compute Triangle Normals with Contiguous Buffer Datastructures
 *
 * @param vertices
 * @param triangles
 * @param triangle_normals              Results will be stored in here
 * @param flip_normals                  If true will flip the normals
 */
void ComputeTriangleNormals(const Matrix<double>& vertices, const std::vector<size_t>& triangles,
                            std::vector<double>& triangle_normals, const bool flip_normals = false);

/**
 * @brief Create a 3D Triangular Mesh (no copy) from the given datastructures
 *
 * @param vertices
 * @param triangles
 * @param halfedges
 * @return HalfEdgeTriangulation
 */
HalfEdgeTriangulation CreateTriMeshFromVectors(std::vector<double>&& vertices, std::vector<size_t>&& triangles,
                                               std::vector<size_t>&& halfedges);

/**
 * @brief Perform Bilateral Filtering on the triangular mesh. Only the normals are smoothed. Neighbors are in 1-ring
 * edge adjecency. \rst :math:`n_o = K \cdot \sum_{j =1}^{N} W_c(||c_i - c_j||) \cdot W_s(|| n_i -n_j||) \cdot n_j`
 * :math:`W_c(||c_i - c_j||) = \operatorname{exp}(-||c_i - c_j||^2/2\sigma_c^2 )`
 * :math:`W_s(||n_i - n_j||) = \operatorname{exp}(-||n_i - n_j||^2/2\sigma_s^2 )`
 * :math:`K = 1 / \sum_{j=1}^{N} W_c(||c_i - c_j||) \cdot W_s(||n_i - n_j||)`
 *
 * \endrst
 *
 * @param mesh
 * @param iterations            Number of iterations
 * @param sigma_length          The standard deviation for exponential decay based on centroid difference between
 * neighbors
 * @param sigma_angle           The standard deviation for exponential decay based on surface normal difference between
 * neighbors
 */
void BilateralFilterNormals(HalfEdgeTriangulation& mesh, int iterations, double sigma_length, double sigma_angle);
// TriMesh CreateTriMeshCopy(const double* vertices_ptr, size_t num_vertices, const size_t* triangles_ptr,
//                           size_t num_triangles);
// TriMesh CreateTriMeshCopy(const double* vertices_ptr, size_t num_vertices, const int* triangles_ptr,
//                           size_t num_triangles);

/**
 * @brief Extacts an organized point cloud from a depth image
 *
 * @param im                    The float depth image. Organized into rows and columns and gives distance along z-axis
 * to surface
 * @param intrinsics            Intrinsics of the camera sensor (3 X 3)
 * @param extrinsics            Extrinsics of the camera, send identity matrix when in doubt (4 X 4)
 * @param stride                Can stride over rows/cols for faster computation
 * @return std::vector<double>
 */
std::vector<double> ExtractPointCloudFromFloatDepth(const Matrix<float>& im, const Matrix<double>& intrinsics,
                                                    const Matrix<double>& extrinsics, const size_t stride);

/**
 * @brief Extracts a Half-Edge Triangulated mesh (Uniform Mesh/Right Cut Mesh) from a depth image
 * Mesh is returned as vertices, triangles, and halfedges
 * TODO: Make this private?
 *
 * @param im                    The float depth image. Organized into rows and columns and gives distance along z-axis
 * to surface
 * @param intrinsics            Intrinsics of the camera sensor (3 X 3)
 * @param extrinsics            Extrinsics of the camera, send identity matrix when in doubt (4 X 4)
 * @param stride                Can stride over rows/cols for faster computation
 * @return std::tuple<std::vector<double>, std::vector<size_t>, std::vector<size_t>>
 */
std::tuple<std::vector<double>, std::vector<size_t>, std::vector<size_t>>
ExtractUniformMeshFromFloatDepth(const Matrix<float>& im, const Matrix<double>& intrinsics,
                                 const Matrix<double>& extrinsics, const size_t stride);

/**
 * @brief Extracts a Half-Edge Triangulated mesh (Uniform Mesh/Right Cut Mesh) from a depth image
 *
 * @param im                    The float depth image. Organized into rows and columns and gives distance along z-axis
 * to surface
 * @param intrinsics            Intrinsics of the camera sensor (3 X 3)
 * @param extrinsics            Extrinsics of the camera, send identity matrix when in doubt (4 X 4)
 * @param stride                Can stride over rows/cols for faster computation
 * @param calc_normals          Will calculate triangle normals if requested
 * @return HalfEdgeTriangulation
 */
HalfEdgeTriangulation ExtractTriMeshFromFloatDepth(const Matrix<float>& im, const Matrix<double>& intrinsics,
                                                   const Matrix<double>& extrinsics, const size_t stride,
                                                   const bool calc_normals = PL_DEFAULT_CALC_NORMALS);

/**
 * @brief Extracts a Half-Edge Triangulated mesh (Uniform Mesh/Right Cut Mesh) from an organized point cloud
 * Returns mesh and T_map. T_map is the valid triangle set that maps between the complete fully connected mesh
 * and all triangles returned. You will mostly likely *not* need T_map and can safely discard.
 *
 * @param points_2D             Organized Point Cloud
 * @param rows                  Rows in organized point clouds
 * @param cols                  Columns in organized point cloud
 * @param stride                Stride Level
 * @param calc_normals          Will calculate triangle normals if requested
 * @return std::tuple<HalfEdgeTriangulation, VUI>
 */
std::tuple<HalfEdgeTriangulation, VUI> ExtractTriMeshFromOrganizedPointCloud(Matrix<double>& points_2D,
                                                                             const size_t rows, const size_t cols,
                                                                             const size_t stride,
                                                                             const bool calc_normals);
/**
 * @brief Special purpose function to taken Open3D format of triangles stored in integers and converts to unsigned
 * intergers using Polylidar3D
 *
 * @param vertices
 * @param triangles
 * @param calc_normals
 * @return HalfEdgeTriangulation
 */
HalfEdgeTriangulation CreateTriMeshCopy(Matrix<double>& vertices, Matrix<int>& triangles,
                                        const bool calc_normals = true);

} // namespace MeshHelper
} // namespace Polylidar

#endif