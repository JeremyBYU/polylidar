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

#define PL_DEFAULT_DIM 2
#define PL_DEFAULT_ALPHA 1.0
#define PL_DEFAULT_XYTHRESH 0.0
#define PL_DEFAULT_LMAX 0.0
#define PL_DEFAULT_MINTRIANGLES 20
#define PL_DEFAULT_MINHOLEVERTICES 3
#define PL_DEFAULT_MINBBOX 100.0
#define PL_DEFAULT_ZTHRESH 0.20
#define PL_DEFAULT_NORMTHRESH 0.90
#define PL_DEFAULT_NORMTHRESH_MIN 0.1
#define PL_DEFAULT_ALLOWEDCLASS 4.0
#define PL_DEFAULT_STRIDE 2
#define PL_DEFAULT_CALC_NORMALS true

#define PL_EPS_RADIAN 0.001

namespace Polylidar {


namespace Utility {

constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();
constexpr std::size_t operator"" _z(unsigned long long n) { return n; }

namespace Math

{



inline double Determinant(const std::array<double, 2> &v1, const std::array<double, 2> &v2)
{
    return v1[0] * v2[1] - v1[1] * v2[0];
}

inline double DotProduct2(const std::array<double, 2> &v1, const std::array<double, 2> &v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1];
}

inline void CrossProduct3(const std::array<double, 3>& u, const std::array<double, 3>& v, double* normal)
{
    // cross product
    normal[0] = u[1] * v[2] - u[2] * v[1];
    normal[1] = u[2] * v[0] - u[0] * v[2];
    normal[2] = u[0] * v[1] - u[1] * v[0];
}

inline void Normalize3(double* normal)
{
    auto norm = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
    normal[0] /= norm;
    normal[1] /= norm;
    normal[2] /= norm;
}

inline double L2Norm(double dx, double dy) { return std::sqrt(dx * dx + dy * dy); }

inline double DotProduct3(const std::array<double, 3>& v1, const std::array<double, 3>& v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

inline double DotProduct3(const double* v1, const std::array<double, 3>& v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}
inline std::array<double, 9> AxisAngleToRotationMatrix(const std::array<double, 3>& axis, const double angle)
{
    std::array<double, 9> rm{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
    // TODO Research what is the proper way to handle this
    if (std::abs(angle) < PL_EPS_RADIAN) return rm;
    auto c = std::cos(angle);
    auto s = std::sin(angle);
    auto t = 1 - c;
    auto& x = axis[0];
    auto& y = axis[1];
    auto& z = axis[2];
    // Creat matrix
    rm[0] = t * x * x + c;
    rm[1] = t * x * y - z * s;
    rm[2] = t * x * z + y * s;

    rm[3] = t * x * y + z * s;
    rm[4] = t * y * y + c;
    rm[5] = t * y * z - x * s;

    rm[6] = t * x * z - y * s;
    rm[7] = t * y * z + x * s;
    rm[8] = t * z * z + c;

    return rm;
}
inline std::tuple<std::array<double, 3>, double> AxisAngleFromVectors(const std::array<double, 3>& v1,
                                                                      const std::array<double, 3>& v2)
{
    std::array<double, 3> axis;
    CrossProduct3(v1, v2, axis.data());
    Normalize3(axis.data());
    double angle = std::acos(DotProduct3(v1, v2));

    return std::make_tuple(axis, angle);
}
// Rotate Vector
inline std::array<double, 3> RotateVector(const double* v1, const std::array<double, 9>& rm)
{
    std::array<double, 3> rv1;
    rv1[0] = rm[0] * v1[0] + rm[1] * v1[1] + rm[2] * v1[2];
    rv1[1] = rm[3] * v1[0] + rm[4] * v1[1] + rm[5] * v1[2];
    rv1[2] = rm[6] * v1[0] + rm[7] * v1[1] + rm[8] * v1[2];

    return rv1;
}

inline double Get360Angle(const std::array<double, 2>& v1, const std::array<double, 2>& v2)
{
    auto dot = DotProduct2(v1, v2);
    auto det = Determinant(v1, v2);
    auto ang = std::atan2(det, dot);
    if (ang < 0)
    {
        ang += M_PI * 2;
    }
    return ang;
}

} // namespace Math

inline void UpdatePlaneDataWithRotationInformation(PlaneData& plane_data)
{
    std::array<double, 3> axis;
    double angle;
    // std::cout << "Normal to Extract: " << PL_PRINT_ARRAY(config.desiredVector) << "; Z Axis: " <<
    // PL_PRINT_ARRAY(DEFAULT_DESIRED_VECTOR) << std::endl;
    std::tie(axis, angle) = Math::AxisAngleFromVectors(plane_data.plane_normal, PL_DEFAULT_DESIRED_VECTOR);
    if (std::abs(angle) > PL_EPS_RADIAN)
    {
        plane_data.need_rotation = true;
        plane_data.rotation_matrix = Math::AxisAngleToRotationMatrix(axis, angle);
    }
}

inline std::vector<PlaneData> CreateMultiplePlaneDataFromNormals(const Matrix<double>& normals)
{
    std::vector<PlaneData> configs;
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
// TODO fix to look at three dimension
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

// Determines if the triangles noise (dz from normal) is below a configurable zThrsh
inline bool CheckZThresh(size_t t, MeshHelper::HalfEdgeTriangulation &mesh, std::array<double, 3> &plane_normal, double &z_thresh)
{
    auto &triangles = mesh.triangles;
    auto &points = mesh.vertices;
    // Get reference to point indices
    auto &pi0 = triangles(t, 0);
    auto &pi1 = triangles(t, 1);
    auto &pi2 = triangles(t, 2);

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

inline bool ValidateTriangle3D(size_t t, MeshHelper::HalfEdgeTriangulation& mesh, double &z_thresh, double &norm_thresh, double &norm_thresh_min, std::array<double, 3> &plane_normal)
{
    auto &normals = mesh.triangle_normals;
    auto normal = &normals(t, 0);

    auto prod = std::abs(Utility::Math::DotProduct3(normal, plane_normal));
    if (prod < norm_thresh_min)
        return false;

    auto passZThresh = CheckZThresh(t, mesh, plane_normal, z_thresh);
    return prod > norm_thresh || passZThresh;
}

class Timer
{
    typedef std::chrono::high_resolution_clock high_resolution_clock;
    typedef std::chrono::microseconds microseconds;

  public:
    explicit Timer(bool run = false):_start()
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