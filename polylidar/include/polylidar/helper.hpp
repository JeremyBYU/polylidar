// This is a helper class for Polylidar
#ifndef POLYLIDARHELPER
#define POLYLIDARHELPER
#define _USE_MATH_DEFINES
#ifndef NDEBUG
    #define NDEBUG
#endif
#include "polylidar/util.hpp"
#include "delaunator.hpp"
#include <cassert>
#include <array>

namespace polylidar {


struct ExtremePoint 
{
    size_t xr_he = -1;
    size_t xr_pi = -1;
    double xr_val = -1 * std::numeric_limits<double>::infinity();
    size_t xl_he = -1;
    size_t xl_pi = -1;
    double xl_val = std::numeric_limits<double>::infinity();

};

double circumsribedRadius(size_t t, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points);

inline double dotProduct3(std::array<double, 3> &v1, std::array<double, 3> &v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

//     Create rotation matrix given an axis and angle
//     https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
std::array<double, 9> axisAngleRotationMatrix(std::array<double, 3> axis, double angle);


inline bool checkPointClass(size_t t, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points, double allowedClass)
{
    auto &triangles = delaunay.triangles;
    std::vector<size_t> pis = {triangles[t * 3], triangles[t * 3 + 1], triangles[t * 3 + 2]};
    size_t &pi0 = pis[0];
    size_t &pi1 = pis[1];
    size_t &pi2 = pis[2];
    // std::cout << "pi0" << pi0 << " pi1" << pi1 << " pi2" << pi0 << std::endl; 
    // std::cout << "pi0" << points(pi0, 3) << " pi1" << points(pi1, 3) << " pi2" << points(pi2, 3) << std::endl; 
    auto result = points(pi0, 3_z) == allowedClass && points(pi1, 3_z) == allowedClass && points(pi2, 3_z) == allowedClass;
    return result;

}

inline void maxZChangeAndNormal(size_t t, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points,
                                double &diff, std::array<double, 3> &normal, std::array<double, 3> &desiredVector) {
    auto &triangles = delaunay.triangles;
    // Get reference to point indices
    auto &pi0 = triangles[t * 3];
    auto &pi1 = triangles[t * 3 + 1];
    auto &pi2 = triangles[t * 3 + 2];

    std::array<double, 3> vv1 = {points(pi0, 0),points(pi0, 1),points(pi0, 2)};
    std::array<double, 3> vv2 = {points(pi1, 0),points(pi1, 1),points(pi1, 2)};
    std::array<double, 3> vv3 = {points(pi2, 0),points(pi2, 1),points(pi2, 2)};

    // two lines of triangle
    // V1 is starting index
    std::array<double, 3> u{{vv2[0] - vv1[0], vv2[1] - vv1[1], vv2[2] - vv1[2]}};
    std::array<double, 3> v{{vv3[0] - vv1[0], vv3[1] - vv1[1], vv3[2] - vv1[2]}};

    // cross product
    normal[0] = u[1] * v[2] - u[2] * v[1];
    normal[1] = u[2] * v[0] - u[0] * v[2];
    normal[2] = u[0] * v[1] - u[1] * v[0];

    auto norm = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);

    normal[0] /= norm;
    normal[1] /= norm;
    normal[2] /= norm;

    // Calculate the noise amt from the desired vector
    auto u_z = dotProduct3(u, desiredVector);
    auto v_z = dotProduct3(v, desiredVector);
    auto s_z = 0.0;

    // get max dimension change in a triangle
    auto zMin = std::min(std::min(u_z, v_z), s_z);
    auto zMax = std::max(std::max(u_z, v_z), s_z);
    diff = zMax - zMin;

}


inline double getMaxDimTriangle(size_t t, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points) {
    auto pi0= delaunay.triangles[t * 3];
    auto pi1 = delaunay.triangles[t * 3 + 1];
    auto pi2 = delaunay.triangles[t * 3 + 2];
    // get max XY dimension change on one side of the triangle
    auto l1 = std::max(std::abs(points(pi0, 0) - points(pi1, 0)), std::abs(points(pi0, 1) - points(pi1, 1)));
    auto l2 = std::max(std::abs(points(pi0, 0) - points(pi2, 0)), std::abs(points(pi0, 1) - points(pi2, 1)));
    auto l3 = std::max(std::abs(points(pi1, 0) - points(pi2, 0)), std::abs(points(pi1, 1) - points(pi2, 1)));
    return std::max(std::max(l1, l2), l3);
}

// inline double l2Norm(double &x0, double &y0, double &x1, double &y1)
// {
//     return l2Norm(x1-x0, y1-y0);
// }

inline double l2Norm(double dx, double dy)
{
    return std::sqrt(dx * dx + dy * dy);
}

inline double getMaxEdgeLength(size_t t, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points) {
    auto pi0= delaunay.triangles[t * 3];
    auto pi1 = delaunay.triangles[t * 3 + 1];
    auto pi2 = delaunay.triangles[t * 3 + 2];
    // get max XY dimension change on one side of the triangle
    auto l1 = l2Norm(points(pi0, 0) - points(pi1, 0), points(pi0, 1) - points(pi1, 1));
    auto l2 = l2Norm(points(pi0, 0) - points(pi2, 0), points(pi0, 1) - points(pi2, 1));
    auto l3 = l2Norm(points(pi1, 0) - points(pi2, 0), points(pi1, 1) - points(pi2, 1));
    return std::max(std::max(l1, l2), l3);
}

double norm(double a, double b);

std::ostream& operator<<(std::ostream& os, const std::array<double, 2ul>& values);
std::ostream& operator<<(std::ostream& os, const std::vector<double>& values);

inline void trackExtremePoint(size_t pi, Matrix<double> &points, ExtremePoint &exPoint, size_t he){
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

inline std::array<double, 2> getVector(size_t edge, delaunator::HalfEdgeTriangulation &delaunay, bool flip=false ){
    auto &coords = delaunay.coords;
    auto &triangles = delaunay.triangles;
    std::array<double, 2> result;

    auto pi = triangles[edge];
    std::array<double, 2> p0 = {coords(pi, 0_z), coords(pi, 1_z)};

    auto piNext = triangles[nextHalfedge(edge)];
    std::array<double, 2> p1 = {coords(piNext, 0_z), coords(piNext, 1_z)};
    if (flip) {
        result[0] = p0[0] - p1[0];
        result[1] = p0[1] - p1[1];
    } else {
        result[0] = p1[0] - p0[0];
        result[1] = p1[1] - p0[1];
    }
    return result; // RVO
}

inline double determinant(const std::array<double, 2> &v1, const std::array<double, 2> &v2) {
  return v1[0] * v2[1] - v1[1] * v2[0];
}

inline double dotProduct2(const std::array<double, 2> &v1, const std::array<double, 2> &v2) {
  return v1[0] * v2[0] + v1[1] * v2[1];
}

inline double get360Angle(const std::array<double, 2> &v1, const std::array<double, 2> &v2) {
    auto dot = dotProduct2(v1, v2);
    auto det = determinant(v1, v2);
    auto ang = std::atan2(det, dot);
    if (ang < 0) {
        ang += M_PI * 2;
    }
    return ang;
}


inline size_t getHullEdge(size_t &incomingEdge, std::vector<size_t> &outgoingEdges,  delaunator::HalfEdgeTriangulation &delaunay, bool isHole=false)
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
        auto min_pos = std::distance(angleDist.begin(),std::min_element(angleDist.begin(),angleDist.end()));
        return outgoingEdges[min_pos];
    } else {
        auto max_pos = std::distance(angleDist.begin(),std::max_element(angleDist.begin(),angleDist.end()));
        return outgoingEdges[max_pos];
    }

}


inline size_t getHullEdgeStart(const std::array<double, 2> &v1, const std::vector<size_t> &outgoingEdges,  delaunator::HalfEdgeTriangulation &delaunay, bool isHole=false)
{
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
        auto min_pos = std::distance(angleDist.begin(),std::min_element(angleDist.begin(),angleDist.end()));
        return outgoingEdges[min_pos];
    } else {
        auto max_pos = std::distance(angleDist.begin(),std::max_element(angleDist.begin(),angleDist.end()));
        return outgoingEdges[max_pos];
    }

}

}



#endif
