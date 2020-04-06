

#include "polylidar/polylidar.hpp"
#include "delaunator.hpp"
#include "npy/npy.h"


void PrintPoint(delaunator::Delaunator &mesh, size_t idx)
{

    std:: cout << "[" << mesh.triangles[idx * 3] << ", " << mesh.triangles[idx * 3 + 1] << ", " << mesh.triangles[idx * 3 + 2] << "]" <<  std::endl;
}

int main(int argc, char const *argv[])
{
    std::vector<unsigned long> shape;
    bool fortran_order;
    std::vector<double> data;
    std::string path = "fixtures/temp/broken.npy";
    npy::LoadArrayFromNumpy(path, shape, fortran_order, data);

    std::ofstream out("fixtures/temp/golden.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!

    std::cout << shape[0] << ", " << shape[1] << std::endl;

    polylidar::Matrix<double> point_mat(data.data(), shape[0], shape[1]);
    // Delaunator::Delaunator mesh(points);
    delaunator::Delaunator mesh(point_mat);
    mesh.triangulate();

    std::cout << "Raw Buffer size:" << mesh.triangles.size() << std::endl;
    std::cout << "Number of triangles:" << mesh.triangles.size() /3 << std::endl;

    PrintPoint(mesh, 0);
    PrintPoint(mesh, 1);
    std::cout << "..." << std::endl;
    PrintPoint(mesh, 49595);
    PrintPoint(mesh, 49596);

    return 0;
}
