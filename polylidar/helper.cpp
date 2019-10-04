#include "helper.hpp"

// namespace py = pybind11;

namespace polylidar {


    double norm(double a, double b) {
        return std::sqrt(a * a + b * b);
    }

    double circumsribedRadius(size_t t, delaunator::Delaunator &delaunay, Matrix &points){
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

}
