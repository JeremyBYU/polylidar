#ifndef MESHHELPER
#define MESHHELPER
#include <vector>
#include <array>

#include <unordered_map>
#include <parallel_hashmap/phmap.h>


namespace MeshHelper

{

#ifdef PL_USE_STD_UNORDERED_MAP
template <typename T, typename G>
using unordered_map = std::unordered_map<T, G>;
#else
template <typename T, typename G>
using unordered_map = phmap::flat_hash_map<T, G>;
#endif

using MatX2I = std::vector<std::array<size_t, 2>>;

inline size_t CantorMapping(const size_t k1, const size_t k2)
{
    auto dk1 = static_cast<double>(k1);
    auto dk2 = static_cast<double>(k2);
    auto mapping = static_cast<size_t>(((dk1 + dk2) * (dk1 + dk2 + 1)) / 2.0 + dk2);
    return mapping;
}

inline std::vector<size_t> ExtractHalfEdges(const std::vector<size_t> &triangles)
{
    // auto before = std::chrono::high_resolution_clock::now();
    size_t max_limit = std::numeric_limits<size_t>::max();

    std::vector<size_t> halfedges(triangles.size(), max_limit);
    MatX2I halfedges_pi(triangles.size());
    unordered_map<size_t, size_t> vertex_indices_to_half_edge_index;
    vertex_indices_to_half_edge_index.reserve(triangles.size());

    for (size_t triangle_index = 0; triangle_index < triangles.size(); triangle_index+=3)
    {
        const size_t* triangle = &triangles[triangle_index];
        size_t &num_half_edges = triangle_index;

        size_t he_0_index = num_half_edges;
        size_t he_1_index = num_half_edges + 1;
        size_t he_2_index = num_half_edges + 2;
        size_t he_0_mapped = CantorMapping(triangle[0], triangle[1]);
        size_t he_1_mapped = CantorMapping(triangle[1], triangle[2]);
        size_t he_2_mapped = CantorMapping(triangle[2], triangle[0]);

        std::array<size_t, 2> &he_0 = halfedges_pi[he_0_index];
        std::array<size_t, 2> &he_1 = halfedges_pi[he_1_index];
        std::array<size_t, 2> &he_2 = halfedges_pi[he_2_index];

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
        size_t &that_he_index = halfedges[this_he_index];
        std::array<size_t, 2> &this_he = halfedges_pi[this_he_index];
        size_t that_he_mapped = CantorMapping(this_he[1], this_he[0]);
        if (that_he_index == max_limit &&
            vertex_indices_to_half_edge_index.find(that_he_mapped) !=
                vertex_indices_to_half_edge_index.end())
        {
            size_t twin_he_index =
                vertex_indices_to_half_edge_index[that_he_mapped];
            halfedges[this_he_index] = twin_he_index;
            halfedges[twin_he_index] = this_he_index;
        }
    }
    return halfedges;
}
} // namespace MeshHelper


#endif