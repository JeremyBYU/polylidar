/**
    simple.cpp
    Purpose: Example of using polylidar in C++
    @author Jeremy Castagno
    @version 05/20/19 
*/
#include <iostream>
#include <sstream>      // std::istringstream
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>    

#include "polylidar/polylidar.hpp"

// Print arrays
template <typename TElem>
std::ostream& operator<<(std::ostream& os, const std::vector<TElem>& vec) {
    auto iter_begin = vec.begin();
    auto iter_end   = vec.end();
    os << "[";
    for (auto iter = iter_begin; iter != iter_end; ++iter) {
        std::cout << ((iter != iter_begin) ? "," : "") << *iter;
    }
    os << "]";
    return os;
}

int main(int argc, char *argv[])
{

  std::cout << "Simple C++ Example of Polylidar" << std::endl;
  std::vector<double> points = {
    0.0, 0.0,
    0.0, 1.0,
    1.0, 1.0,
    1.0, 0.0,
    5.0, 0.1,
  };


  // 5 X 2 matrix as one contigious array
  // Convert to multidimensional array
  std::vector<std::size_t> shape = { points.size() / 2, 2 };
  polylidar::Matrix<double> points_(points.data(), shape[0], shape[1]);
  // Set configuration parameters
  polylidar::Config config;
  config.dim = 2;
  config.xyThresh = 0.0;
  config.alpha = 0.0;
  config.lmax = 2.0;
  config.minTriangles = 1;

  // Extract polygon
  std::vector<float> timings;
  auto before = std::chrono::high_resolution_clock::now();
  auto polygons = polylidar::ExtractPolygonsAndTimings(points_, config, timings);
  auto after = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
  std::cout << "Polylidar took " << elapsed.count() << " milliseconds processing a " << shape[0] << " point cloud" << std::endl;
  std::cout << "Point indices of Polygon Shell: ";
  for(auto const& polygon: polygons) {
    std::cout << polygon.shell << std::endl;
  }

  std::cout << std::endl;
  std::cout << "Detailed timings in milliseconds:" << std::endl;
  std::cout << std::fixed << std::setprecision(2) << "Delaunay Triangulation: " << timings[0] << "; Mesh Extraction: " << timings[1] << "; Polygon Extraction: " << timings[2] <<std::endl;


  return 0;
}