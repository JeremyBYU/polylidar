#include "polylidar/helper.hpp"
#include "polylidar/polylidar.hpp"

namespace polylidar {


    double norm(double a, double b) {
        return std::sqrt(a * a + b * b);
    }

    std::array<double, 9> axisAngleToRotationMatrix(const std::array<double, 3> &axis, const double angle)
    {
        std::array<double, 9> rm{{1,0,0,
                                0,1,0,
                                0,0,1}};
        // TODO Research what is the proper way to handle this
        if (std::abs(angle) < EPS_RADIAN)
            return rm;
        auto c = cos(angle);
        auto s = sin(angle);
        auto t = 1 -c;
        auto &x = axis[0];
        auto &y = axis[1];
        auto &z = axis[2];
        // Creat matrix
        rm[0] = t*x*x + c;
        rm[1] = t*x*y - z*s;
        rm[2] = t*x*z + y*s;

        rm[3] = t*x*y + z*s;
        rm[4] = t*y*y + c;
        rm[5] = t*y*z - x*s;

        rm[6] = t*x*z - y*s;
        rm[7] = t*y*z + x*s;
        rm[8] = t*z*z + c;

        return rm;
    }
    // Should be unit vectors
    std::tuple<std::array<double, 3>, double> axisAngleFromVectors(const std::array<double, 3> &v1, const std::array<double, 3> &v2)
    {
        std::array<double, 3> axis;
        crossProduct3(v1, v2, axis);
        normalize3(axis);
        double angle = acos(dotProduct3(v1, v2));

        return std::make_tuple(axis, angle);
    }

    // Rotate Vector
    std::array<double, 3> rotateVector(const double *v1, const std::array<double, 9> &rm)
    {
        std::array<double,3> rv1;
        rv1[0] = rm[0] * v1[0] + rm[1] * v1[1] + rm[2] * v1[2];
        rv1[1] = rm[3] * v1[0] + rm[4] * v1[1] + rm[5] * v1[2];
        rv1[2] = rm[6] * v1[0] + rm[7] * v1[1] + rm[8] * v1[2];

        return rv1;
    }

    double circumsribedRadius(size_t t, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points){
        auto pa = delaunay.triangles[t * 3];
        auto pb = delaunay.triangles[t * 3 + 1];
        auto pc = delaunay.triangles[t * 3 + 2];

        auto aLength = norm(points(pa, 0) - points(pb, 0), points(pa, 1) - points(pb, 1));
        auto bLength = norm(points(pb, 0) - points(pc, 0), points(pb, 1) - points(pc, 1));
        auto cLength = norm(points(pc, 0) - points(pa, 0), points(pc, 1) - points(pa, 1));

        auto s = (aLength + bLength + cLength) / 2.0;
        auto area = std::sqrt(s * (s - aLength) * (s - bLength) * (s - cLength));
        return (aLength * bLength * cLength) / (area * 4.0);
    }

    std::ostream& operator<<(std::ostream& os, const std::array<double, 2ul>& values)
    {   
        for (auto &&val : values) {
            os << val << ", ";
        }

        return os;
    }

    std::ostream& operator<<(std::ostream& os, const std::vector<double>& values)
    {
        for (auto &&val : values) {
            os << val << ", ";
        }

        return os;
    }

        std::ostream &operator<<(std::ostream &os, const Config &config)
    {
        os << "Dim=" << config.dim << " alpha=" << config.alpha << " xyThresh=" << config.xyThresh << " lmax=" << config.xyThresh << " minTriangles=" << config.minTriangles
        << " minBboxArea=" << config.minBboxArea << " zThresh=" << config.zThresh << " normThresh=" << config.normThresh
        << " allowedClass=" << config.allowedClass
        << " desiredVector= [" << (config.desiredVector)[0] << ", " << (config.desiredVector)[1] << ", " << (config.desiredVector)[2] << "]";

        return os;
    }

    std::ostream &operator<<(std::ostream &os, const std::vector<size_t> &values)
    {
        os << "[";
        for (auto &&val : values)
        {
            os << val << ", ";
        }
        os << "]";

        return os;
    }

    std::ostream &operator<<(std::ostream &os, const ExtremePoint &values)
    {
        os << "xr_he" << values.xr_he << " xr_pi" << values.xr_pi << " xr_val" << values.xr_val;

        return os;
    }

}
