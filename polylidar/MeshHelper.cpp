#include "polylidar/Mesh/MeshHelper.hpp"
namespace polylidar
{

namespace MeshHelper
{

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

} // namespace MeshHelper
} // namespace polylidar
