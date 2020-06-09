// Work in progress bilateral filter
// I did not put a lot of effort into this implementation
#include "Polylidar/Mesh/MeshHelper.hpp"
#include <iostream>
#include <cmath>
#include "eigen3/Eigen/Dense"
#include "FastExp/fastexp.h"

#define PL_OMP_MAX_THREAD_CENTROIDS 8
#define PL_OMP_MAX_THREAD_BILATERAL 8

using RowMatrixX3d = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

namespace Polylidar {

namespace MeshHelper {

constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();

namespace BilateralCore

{

inline double GaussianWeight(double value, double sigma_squared)
{
    return fastexp::exp<double, fastexp::Product, 10>(-(value * value) / sigma_squared);
    // return std::exp(-(value * value) / sigma_squared);
}

inline void IntegrateTriangle(Eigen::Block<Eigen::Ref<Eigen::Map<RowMatrixX3d>>, 1, 3, true>& normal,
                              Eigen::Block<Eigen::Ref<Eigen::Map<RowMatrixX3d>>, 1, 3, true>& centroid,
                              Eigen::Block<Eigen::Ref<Eigen::Map<RowMatrixX3d>>, 1, 3, true>& nbr_normal,
                              Eigen::Block<Eigen::Ref<Eigen::Map<RowMatrixX3d>>, 1, 3, true>& nbr_centroid,
                              double& total_weight, Eigen::Vector3d& sum_normal, double& sas, double& sls)
{
    auto normal_dist = (nbr_normal - normal).norm();
    auto centroid_dist = (nbr_centroid - centroid).norm();

    if (std::isnan(centroid_dist)) return;

    auto weight = GaussianWeight(normal_dist, sas) * GaussianWeight(centroid_dist, sls);
    total_weight += weight;
    sum_normal += weight * nbr_normal;
}

inline void SmoothNormal(Eigen::Ref<Eigen::Map<RowMatrixX3d>> normals_in, Eigen::Ref<Eigen::Map<RowMatrixX3d>> centroids,
                  Matrix<size_t> &halfedges, Eigen::Ref<Eigen::Map<RowMatrixX3d>> normals_out, double sls, double sas,
                  int t)
{
    double total_weight = 0.0;
    Eigen::Vector3d sum_normal(0, 0, 0);

    auto he1 = halfedges(t, 0);
    auto he2 = halfedges(t, 1);
    auto he3 = halfedges(t, 2);

    auto normal = normals_in.row(t);
    auto centroid = centroids.row(t);

    if (he1 != INVALID_INDEX)
    {
        auto normal1 = normals_in.row(static_cast<int>(he1 / 3));
        auto centroid1 = centroids.row(static_cast<int>(he1 / 3));
        IntegrateTriangle(normal, centroid, normal1, centroid1, total_weight, sum_normal, sas, sls);
    }
    if (he2 != INVALID_INDEX)
    {
        auto normal2 = normals_in.row(static_cast<int>(he2 / 3));
        auto centroid2 = centroids.row(static_cast<int>(he2 / 3));
        IntegrateTriangle(normal, centroid, normal2, centroid2, total_weight, sum_normal, sas, sls);
    }
    if (he3 != INVALID_INDEX)
    {
        auto normal3 = normals_in.row(static_cast<int>(he3 / 3));
        auto centroid3 = centroids.row(static_cast<int>(he3 / 3));
        IntegrateTriangle(normal, centroid, normal3, centroid3, total_weight, sum_normal, sas, sls);
    }

    if (total_weight > 0)
    {
        normals_out.row(t) = sum_normal / total_weight;
    }
}

void BilateralNormalLoop(Matrix<double>& normals_in, Matrix<double>& centroids, Matrix<size_t> &halfedges,
                         Matrix<double>& normals_out, double sls, double sas)
{
    const int rows = static_cast<int>(normals_in.rows);

    Eigen::Map<RowMatrixX3d> normals_in_e(normals_in.ptr, normals_in.rows, 3);
    Eigen::Map<RowMatrixX3d> centroids_e(centroids.ptr, centroids.rows, 3);
    Eigen::Map<RowMatrixX3d> normals_out_e(normals_out.ptr, normals_out.rows, 3);

#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), PL_OMP_MAX_THREAD_BILATERAL);
    num_threads = std::max(num_threads, 1);
#pragma omp parallel for schedule(guided) num_threads(num_threads)
#endif
    for (int t = 0; t < rows; ++t)
    {
        // std::cout << t << std::endl;
        SmoothNormal(normals_in_e, centroids_e, halfedges, normals_out_e, sls, sas, t);
    }
}

} // namespace BilateralCore


void ComputeCentroids(HalfEdgeTriangulation& mesh, Matrix<double>& centroids)
{
    int num_tris = static_cast<int>(mesh.triangles.rows);

    auto& centroid_data = centroids.data;
    centroids.rows = num_tris;
    centroids.cols = 3;

    auto& triangles = mesh.triangles;

#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), PL_OMP_MAX_THREAD_CENTROIDS);
    num_threads = std::max(num_threads, 1);
#pragma omp parallel for schedule(guided) num_threads(num_threads)
#endif
    for (int t = 0; t < num_tris; ++t)
    {
        auto p1 = triangles(t, 0);
        auto p2 = triangles(t, 1);
        auto p3 = triangles(t, 2);

        centroid_data[t * 3] = (mesh.vertices(p1, 0) + mesh.vertices(p2, 0) + mesh.vertices(p3, 0)) / 3.0;
        centroid_data[t * 3 + 1] = (mesh.vertices(p1, 1) + mesh.vertices(p2, 1) + mesh.vertices(p3, 1)) / 3.0;
        centroid_data[t * 3 + 2] = (mesh.vertices(p1, 2) + mesh.vertices(p2, 2) + mesh.vertices(p3, 2)) / 3.0;
    }
}

void BilateralFilterNormals(HalfEdgeTriangulation& mesh, int iterations, double sigma_length,
                                      double sigma_angle)
{
    auto& normals = mesh.triangle_normals;
    auto& halfedges = mesh.halfedges;
    // Create new datastructures
    Matrix<double> new_normals(normals); // copy
    Matrix<double> centroids;

    centroids.data.resize(normals.rows * 3);
    centroids.ptr = centroids.data.data();

    ComputeCentroids(mesh, centroids);

    double sigma_length_squared = 2.0f * sigma_length * sigma_length;
    double sigma_angle_squared = 2.0f * sigma_angle * sigma_angle;

    bool need_copy = false;
    for (int i = 0; i < iterations; ++i)
    {
        if (i % 2 == 0)
        {
            BilateralCore::BilateralNormalLoop(normals, centroids, halfedges, new_normals, sigma_length_squared, sigma_angle_squared);
            need_copy = false;
        }
        else
        {
            BilateralCore::BilateralNormalLoop(new_normals, centroids, halfedges, normals, sigma_length_squared, sigma_angle_squared);
            need_copy = true;
        }
    }

    if (!need_copy)
    {
        normals.data.swap(new_normals.data);
        normals.ptr = normals.data.data();
    }

}

} // namespace MeshHelper

} // namespace Polylidar