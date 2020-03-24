#ifndef POLYLIDAR_UTILITY
#define POLYLIDAR_UTILITY

#include <iostream>
#include <cstddef>
#include <vector>
#include <cmath>
#include <tuple>

#define PL_EPS_RADIAN 0.001

namespace Polylidar {

namespace Utility {

constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();

namespace Math

{
    inline void CrossProduct3(const std::array<double, 3> &u, const std::array<double, 3> &v, double *normal)
    {
        // cross product
        normal[0] = u[1] * v[2] - u[2] * v[1];
        normal[1] = u[2] * v[0] - u[0] * v[2];
        normal[2] = u[0] * v[1] - u[1] * v[0];
    }

    inline void Normalize3(double *normal)
    {
        auto norm = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
        normal[0] /= norm;
        normal[1] /= norm;
        normal[2] /= norm;
    }

    inline double DotProduct3(const std::array<double, 3> &v1, const std::array<double, 3> &v2)
    {
        return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
    }

    inline double DotProduct3(const double *v1, const std::array<double, 3> &v2)
    {
        return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
    }
    inline std::array<double, 9> AxisAngleToRotationMatrix(const std::array<double, 3> &axis, const double angle)
    {
        std::array<double, 9> rm{{1, 0, 0,
                                0, 1, 0,
                                0, 0, 1}};
        // TODO Research what is the proper way to handle this
        if (std::abs(angle) < PL_EPS_RADIAN)
            return rm;
        auto c = std::cos(angle);
        auto s = std::sin(angle);
        auto t = 1 - c;
        auto &x = axis[0];
        auto &y = axis[1];
        auto &z = axis[2];
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
    inline std::tuple<std::array<double, 3>, double> AxisAngleFromVectors(const std::array<double, 3> &v1, const std::array<double, 3> &v2)
    {
        std::array<double, 3> axis;
        CrossProduct3(v1, v2, axis.data());
        Normalize3(axis.data());
        double angle = std::acos(DotProduct3(v1, v2));

        return std::make_tuple(axis, angle);
    }
    // Rotate Vector
    inline std::array<double, 3> RotateVector(const double *v1, const std::array<double, 9> &rm)
    {
        std::array<double, 3> rv1;
        rv1[0] = rm[0] * v1[0] + rm[1] * v1[1] + rm[2] * v1[2];
        rv1[1] = rm[3] * v1[0] + rm[4] * v1[1] + rm[5] * v1[2];
        rv1[2] = rm[6] * v1[0] + rm[7] * v1[1] + rm[8] * v1[2];

        return rv1;
    }
}



inline void UpdatePlaneDataWithRotationInformation(PlaneData &plane_data)
{
    std::array<double, 3> axis;
    double angle;
    // std::cout << "Normal to Extract: " << PL_PRINT_ARRAY(config.desiredVector) << "; Z Axis: " << PL_PRINT_ARRAY(DEFAULT_DESIRED_VECTOR) << std::endl;
    std::tie(axis, angle) = Math::AxisAngleFromVectors(plane_data.plane_normal, PL_DEFAULT_DESIRED_VECTOR);
    if (std::abs(angle) > PL_EPS_RADIAN)
    {
        plane_data.need_rotation = true;
        plane_data.rotation_matrix = Math::AxisAngleToRotationMatrix(axis, angle);
    }
}

inline std::vector<PlaneData> CreateMultiplePlaneDataFromNormals(const Matrix<double> &normals)
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

}

} // namespace Polylidar

#endif