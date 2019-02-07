
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

double norm(double a, double b);

inline void trackExtremePoint(size_t pi, pybind11::detail::unchecked_reference<double, 2L> &points, ExtremePoint &exPoint, size_t he); 


// function maxPoint(
//   pi: number,
//   delaunay: Delaunator<number>,
//   extremePoint: IExtremePoint,
//   he: number,
// ) {
//   const points = delaunay.coords

//   if (points[pi * 2] > extremePoint.xr.val) {
//     extremePoint.xr.val = points[pi * 2]
//     extremePoint.xr.he = he
//     extremePoint.xr.pi = pi
//   }

//   if (points[pi * 2] < extremePoint.xl.val) {
//     extremePoint.xl.val = points[pi * 2]
//     extremePoint.xl.he = he
//     extremePoint.xl.pi = pi
//   }
// }

}



#endif
