#ifndef POLYLIDAR_UTILITY
#define POLYLIDAR_UTILITY

#include <iostream>
#include <cstddef>
#include <vector>
#include <cmath>
#include <tuple>
#include <chrono>
#include <unordered_map>
#include "Polylidar/Mesh/MeshHelper.hpp"
#include "Polylidar/UtilityMath.hpp"

namespace Polylidar {

std::string GetPolylidarVersion();
bool RobustPredicatesActivated();

namespace Utility {

constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();
constexpr std::size_t operator"" _z(unsigned long long n) { return n; }

inline void UpdatePlaneDataWithRotationInformation(PlaneData& plane_data)
{
    std::array<double, 3> axis;
    double angle;
    // std::cout << "Normal to Extract: " << PL_PRINT_ARRAY(config.desiredVector) << "; Z Axis: " <<
    // PL_PRINT_ARRAY(DEFAULT_DESIRED_VECTOR) << std::endl;
    std::tie(axis, angle) = Math::AxisAngleFromVectors(plane_data.plane_normal, PL_DEFAULT_DESIRED_VECTOR);
    // std::cout << "Axis Angle" << axis[0] << "," << axis[1] << ", " << axis[2] << "; " << angle << std::endl;

    // Check if axis angle is malformed
    if (std::isnan(axis[0]))
    {
        // Malformed Cross Product, Vectors must be parallel or anti parallel
        if (std::abs(angle) > 1.0)
        {
            // Vectors are opposite to each other!
            plane_data.need_rotation = true;
            plane_data.rotation_matrix = {{-1, 0, 0, 0, -1, 0, 0, 0, -1}};
        }
        else
        {
            // Vectors are aligned
            plane_data.need_rotation = false;
        }
    }
    else
    {
        if (std::abs(angle) > PL_EPS_RADIAN)
        {
            plane_data.need_rotation = true;
            plane_data.rotation_matrix = Math::AxisAngleToRotationMatrix(axis, angle);
        }
    }
}

inline std::vector<PlaneData> CreateMultiplePlaneDataFromNormals(const Matrix<double>& normals)
{
    std::vector<PlaneData> configs;
    if (normals.rows > 253)
    {
        throw std::domain_error("A max of 254 unit normals are allowed. Please reduce the amount of normals.");
    }
    for (size_t i = 0; i < normals.rows; i++)
    {
        PlaneData plane_data;
        // Copy normal information into the new config
        plane_data.plane_normal[0] = normals(i, 0);
        plane_data.plane_normal[1] = normals(i, 1);
        plane_data.plane_normal[2] = normals(i, 2);
        plane_data.normal_id = static_cast<uint8_t>(i + 1);
        UpdatePlaneDataWithRotationInformation(plane_data);
        configs.emplace_back(std::move(plane_data));
    }
    return configs;
}

inline double GetMaxEdgeLength(size_t t, MeshHelper::HalfEdgeTriangulation& mesh)
{
    auto& triangles = mesh.triangles;
    auto& points = mesh.vertices;
    auto& pa = triangles(t, 0_z);
    auto& pb = triangles(t, 1_z);
    auto& pc = triangles(t, 2_z);
    // get max length of triangle
    auto l1 = Math::L2Norm(points(pa, 0) - points(pb, 0), points(pa, 1) - points(pb, 1));
    auto l2 = Math::L2Norm(points(pb, 0) - points(pc, 0), points(pb, 1) - points(pc, 1));
    auto l3 = Math::L2Norm(points(pc, 0) - points(pa, 0), points(pc, 1) - points(pa, 1));
    return std::max(std::max(l1, l2), l3);
}

inline double GetMaxEdgeLength3D(size_t t, MeshHelper::HalfEdgeTriangulation& mesh)
{
    auto& triangles = mesh.triangles;
    auto& points = mesh.vertices;
    auto& pa = triangles(t, 0_z);
    auto& pb = triangles(t, 1_z);
    auto& pc = triangles(t, 2_z);
    // get max length of triangle
    auto l1 = Math::L2Norm3D(points(pa, 0) - points(pb, 0), points(pa, 1) - points(pb, 1), points(pa, 2) - points(pb, 2));
    auto l2 = Math::L2Norm3D(points(pb, 0) - points(pc, 0), points(pb, 1) - points(pc, 1), points(pb, 2) - points(pc, 2));
    auto l3 = Math::L2Norm3D(points(pc, 0) - points(pa, 0), points(pc, 1) - points(pa, 1), points(pc, 2) - points(pa, 2));
    return std::max(std::max(l1, l2), l3);
}

inline bool GetAllVertexClasses(size_t t, MeshHelper::HalfEdgeTriangulation& mesh)
{
    auto& triangles = mesh.triangles;
    auto& pa = triangles(t, 0_z);
    auto& pb = triangles(t, 1_z);
    auto& pc = triangles(t, 2_z);
    // get max length of triangle
    return mesh.vertex_classes(pa) && mesh.vertex_classes(pb) && mesh.vertex_classes(pc);
}

inline double CircumsribedRadius(size_t t, MeshHelper::HalfEdgeTriangulation& mesh)
{
    auto& triangles = mesh.triangles;
    auto& points = mesh.vertices;
    auto& pa = triangles(t, 0_z);
    auto& pb = triangles(t, 1_z);
    auto& pc = triangles(t, 2_z);

    auto aLength = Math::L2Norm(points(pa, 0) - points(pb, 0), points(pa, 1) - points(pb, 1));
    auto bLength = Math::L2Norm(points(pb, 0) - points(pc, 0), points(pb, 1) - points(pc, 1));
    auto cLength = Math::L2Norm(points(pc, 0) - points(pa, 0), points(pc, 1) - points(pa, 1));

    auto s = (aLength + bLength + cLength) / 2.0;
    auto area = std::sqrt(s * (s - aLength) * (s - bLength) * (s - cLength));
    return (aLength * bLength * cLength) / (area * 4.0);
}

inline bool ValidateTriangle2D(size_t t, MeshHelper::HalfEdgeTriangulation& mesh, double alpha = 1.0, double lmax = 0.0)
{
    // auto maxXY = getMaxDimTriangle(t, delaunay, points);
    // std::cout << "Triangle " << t << " Radius: " << radius << std::endl;
    if (alpha > 0.0 && CircumsribedRadius(t, mesh) > alpha)
    {
        return false;
    }
    else if (lmax > 0.0 && GetMaxEdgeLength(t, mesh) > lmax)
    {
        return false;
    }

    return true;
}

inline bool ValidateTriangleLength(size_t t, MeshHelper::HalfEdgeTriangulation& mesh, double lmax = 0.0)
{
    return GetMaxEdgeLength3D(t, mesh) <= lmax;
}

// Determines if the triangles noise (dz from normal) is below a configurable zThrsh
inline bool CheckZThresh(size_t t, MeshHelper::HalfEdgeTriangulation& mesh, std::array<double, 3>& plane_normal,
                         double& z_thresh)
{
    auto& triangles = mesh.triangles;
    auto& points = mesh.vertices;
    // Get reference to point indices
    auto& pi0 = triangles(t, 0);
    auto& pi1 = triangles(t, 1);
    auto& pi2 = triangles(t, 2);

    std::array<double, 3> vv1 = {points(pi0, 0), points(pi0, 1), points(pi0, 2)};
    std::array<double, 3> vv2 = {points(pi1, 0), points(pi1, 1), points(pi1, 2)};
    std::array<double, 3> vv3 = {points(pi2, 0), points(pi2, 1), points(pi2, 2)};

    // two lines of triangle
    // V1 is starting index
    std::array<double, 3> u{{vv2[0] - vv1[0], vv2[1] - vv1[1], vv2[2] - vv1[2]}};
    std::array<double, 3> v{{vv3[0] - vv1[0], vv3[1] - vv1[1], vv3[2] - vv1[2]}};

    // Calculate the noise amt from the desired vector
    auto u_z = Utility::Math::DotProduct3(u, plane_normal);
    auto v_z = Utility::Math::DotProduct3(v, plane_normal);
    auto s_z = 0.0;

    // get max dimension change in a triangle
    auto zMin = std::min(std::min(u_z, v_z), s_z);
    auto zMax = std::max(std::max(u_z, v_z), s_z);
    double zDiff = zMax - zMin;
    return z_thresh > 0.0 && zDiff < z_thresh;
}

inline bool ValidateTriangle3D(size_t t, MeshHelper::HalfEdgeTriangulation& mesh, double& z_thresh, double& norm_thresh,
                               double& norm_thresh_min, std::array<double, 3>& plane_normal)
{
    auto& normals = mesh.triangle_normals;
    auto normal = &normals(t, 0);

    auto prod = std::abs(Utility::Math::DotProduct3(normal, plane_normal));
    if (prod < norm_thresh_min) return false;

    auto passZThresh = CheckZThresh(t, mesh, plane_normal, z_thresh);
    return prod > norm_thresh || passZThresh;
}

class Timer
{
    typedef std::chrono::high_resolution_clock high_resolution_clock;
    typedef std::chrono::microseconds microseconds;

  public:
    explicit Timer(bool run = false) : _start()
    {
        if (run) Reset();
    }
    void Reset() { _start = high_resolution_clock::now(); }
    microseconds Elapsed() const
    {
        return std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - _start);
    }
    template <typename T, typename Traits>
    friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const Timer& timer)
    {
        return out << timer.Elapsed().count();
    }

  private:
    high_resolution_clock::time_point _start;
};

} // namespace Utility

} // namespace Polylidar

#endif