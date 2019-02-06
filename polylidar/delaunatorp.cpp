
#include "delaunatorp.hpp"

namespace delaunator {

DelaunatorP::DelaunatorP(pybind11::array_t<double> nparray):Delaunator(coords2) {
    std::cout << "Inside DelaunatorP Constructor" << std::endl;
    auto shape = nparray.shape();
    auto rows = shape[0];
    auto cols = shape[1];

    std::cout << "Shape " << rows << " , " << cols << std::endl;

    auto size = rows * 2;
    const double *data = nparray.data();
    coords2 = std::vector<double>(data, data + size);
}
}

