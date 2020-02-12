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

#define MAX_ITER 50

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

// Read in an input file of the point cloud
bool file_input(std::vector<double> &points, std::string file_path)
{
  std::ifstream is(file_path, std::ios::in);
  if (is.fail())
  {
    std::cerr << "unable to open file for input" << std::endl;
    return false;
  }

  std::string line;
  std::getline(is, line); // Skip header
  // std::cout<<line<<std::endl;
  while (std::getline(is, line))
  {
    std::istringstream iss(line);
    char _;
    double a, b, c;
    if (!(iss >> a >> _ >> b >> _ >> c))
    {
      break;
    } // error
    points.push_back(a);
    points.push_back(b);
    points.push_back(c);
  }

  return true;
}

int main(int argc, char *argv[])
{

  std::cout << "Simple C++ Example of Polylidar" << std::endl;
  std::vector<double> points;
  std::string file_path = "./tests/fixtures/100K_array_3d.csv";

  // N X 3 array as one contigious array
  auto success = file_input(points, file_path);
  if (!success)
    return 0;

  // Convert to multidimensional array
  std::vector<std::size_t> shape = { points.size() / 3, 3 };
  polylidar::Matrix<double> points_(points.data(), shape[0], shape[1]);
  // Set configuration parameters
  polylidar::Config config;
  config.xyThresh = 0.0;
  config.alpha = 0.0;
  config.lmax = 100.0;

  // Extract polygon
  std::vector<float> timings;
  auto before = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < MAX_ITER; i++)
  {
    auto polygons = polylidar::ExtractPolygonsAndTimings(points_, config, timings);
  }

  // Get means for individual timings
  float delaunay_timing = 0.0;
  float mesh_timing = 0.0;
  float polygon_timing = 0.0;

  for (int i = 0; i < timings.size(); i+=3)
  {
    delaunay_timing += timings[i];
    mesh_timing += timings[i+1];
    polygon_timing += timings[i+2];
  }

  delaunay_timing /= MAX_ITER;
  mesh_timing /= MAX_ITER;
  polygon_timing /= MAX_ITER;

  std::cout << "Detailed timings in milleseconds:" << std::endl;
  std::cout << std::fixed << std::setprecision(2) << "Delaunay Triangulation: " << delaunay_timing << "; Mesh Extraction: " << mesh_timing << "; Polygon Extraction: " << polygon_timing <<std::endl;


  return 0;
}