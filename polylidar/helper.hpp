
#ifndef POLYLIDARHELPER
#define POLYLIDARHELPER

#include "delaunator.hpp"
#include "polylidar.hpp"

namespace polylidar {

struct ExtremePoint 
{
    size_t xr_he = -1;
    size_t xr_pi = -1;
    double xr_val = std::numeric_limits<double>::min();
    size_t xl_he = -1;
    size_t xl_pi = -1;
    double xl_val = std::numeric_limits<double>::max();

};

double circumsribedRadius(size_t t, delaunator::Delaunator &delaunay, pybind11::detail::unchecked_reference<double, 2L> &points);

inline double getMaxDimTriangle(size_t t, delaunator::Delaunator &delaunay, pybind11::detail::unchecked_reference<double, 2L> &points) {
    auto &triangles = delaunay.triangles;
    std::vector<size_t> pis = {triangles[t * 3], triangles[t * 3 + 1], triangles[t * 3 + 2]};
    auto &pi0 = pis[0];
    auto &pi1 = pis[1];
    auto &pi2 = pis[2];
    // get max XY dimension change on one side of the triangle
    auto l1 = std::max(std::abs(points(pi0, 0) - points(pi1, 0)), std::abs(points(pi0, 1) - points(pi1, 1)));
    auto l2 = std::max(std::abs(points(pi0, 0) - points(pi2, 0)), std::abs(points(pi0, 1) - points(pi2, 1)));
    auto l3 = std::max(std::abs(points(pi1, 0) - points(pi2, 0)), std::abs(points(pi1, 1) - points(pi2, 1)));
    return std::max(std::max(l1, l2), l3);
}

double norm(double a, double b);

std::ostream& operator<<(std::ostream& os, const std::array<double, 2ul>& values);
std::ostream& operator<<(std::ostream& os, const std::vector<double>& values);

inline void trackExtremePoint(size_t pi, pybind11::detail::unchecked_reference<double, 2L> &points, ExtremePoint &exPoint, size_t he){
    
    if (points(pi,0) > exPoint.xr_val) {
        exPoint.xr_he = he;
        exPoint.xr_pi = pi;
        exPoint.xr_val = points(pi, 0);
    } else if(points(pi,1) < exPoint.xl_val) {
        exPoint.xl_he = he;
        exPoint.xl_pi = pi;
        exPoint.xl_val = points(pi, 1);
    }

}

inline size_t fast_mod(const size_t i, const size_t c) {
    return i >= c ? i % c : i;
}


inline size_t nextHalfedge(size_t e) {
  return fast_mod(e, 3) == 2 ? e - 2 : e + 1;
}

inline std::array<double, 2> getVector(size_t edge, delaunator::Delaunator &delaunay, bool flip=false ){
    auto &coords = delaunay.coords;
    auto &triangles = delaunay.triangles;
    std::array<double, 2> result;

    auto pi = triangles[edge];
    std::array<double, 2> p0 = {coords[pi * 2], coords[pi * 2 +1]};

    auto piNext = triangles[nextHalfedge(edge)];
    std::array<double, 2> p1 = {coords[piNext * 2], coords[piNext * 2 +1]};
    if (flip) {
        result[0] = p0[0] - p1[0];
        result[1] = p0[1] - p1[1];
    } else {
        result[0] = p1[0] - p0[0];
        result[1] = p1[1] - p0[1];
    }
    return result; // RVO
}

inline double determinant(std::array<double, 2> &v1, std::array<double, 2> &v2) {
  return v1[0] * v2[1] - v1[1] * v2[0];
}

inline double dotProduct2(std::array<double, 2> &v1, std::array<double, 2> &v2) {
  return v1[0] * v2[0] + v1[1] * v2[1];
}

inline double get360Angle(std::array<double, 2> &v1, std::array<double, 2> &v2) {
    auto dot = dotProduct2(v1, v2);
    auto det = determinant(v1, v2);
    auto ang = std::atan2(det, dot);
    if (ang < 0) {
        ang += M_PI * 2;
    }
    return ang;
}


inline size_t getHullEdge(size_t &incomingEdge, std::vector<size_t> &outgoingEdges,  delaunator::Delaunator &delaunay, bool isHole=false)
{
    auto v1 = getVector(incomingEdge, delaunay, true);
    // std::cout << "v1: " << v1 << std::endl; 
    std::vector<std::array<double, 2>> otherVectors;
    // Gosh everything is so verbose with c++, even with the c11+ stdlib
    std::transform(outgoingEdges.begin(), outgoingEdges.end(), std::back_inserter(otherVectors), 
                    [&delaunay](size_t edge) -> std::array<double,2> {return getVector(edge, delaunay, false);});

    // for (auto &&vecs : otherVectors) {
    //     std::cout << "other vec " << vecs << std::endl;
    // }
    
    std::vector<double> angleDist;
    std::transform(otherVectors.begin(), otherVectors.end(), std::back_inserter(angleDist), 
                [&v1](std::array<double,2> &outVector) -> double {return get360Angle(v1, outVector);});
    
    // for (auto &&angle : angleDist) {
    //     std::cout << "angle " << angle << std::endl;
    // }

    // YOUR SELECTING ANGLE
    if (isHole) {
        int min_pos = std::distance(angleDist.begin(),std::min_element(angleDist.begin(),angleDist.end()));
        return outgoingEdges[min_pos];
    } else {
        int max_pos = std::distance(angleDist.begin(),std::max_element(angleDist.begin(),angleDist.end()));
        return outgoingEdges[max_pos];
    }

}

}



#endif
