#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include "polylidar.hpp"


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
  int n = 0;
  std::ifstream is(file_path, std::ios::in);
  if (is.fail())
  {
    std::cerr << "unable to open file for input" << std::endl;
    return false;
  }

  std::string line;
  std::getline(is, line);
  std::cout<<line<<std::endl;
  while (std::getline(is, line))
  {
    std::istringstream iss(line);
    char _;
    double a, b, c, d;
    if (!(iss >> a >> _ >> b >> _ >> c >> _ >> d))
    {
      break;
    } // error
    points.push_back(a);
    points.push_back(b);
    points.push_back(c);
    points.push_back(d);
    n += 4;
    // process pair (a,b)
  }

  return true;
}

int main(int argc, char *argv[])
{

  std::cout << "Simple C++ Example of Polylidar" << std::endl;
  std::vector<double> points;
  std::string file_path = "../../tests/fixtures/building1.csv";

  // N X 4 array as one contigous array
  auto success = file_input(points, file_path);
  if (!success)
    return 0;

  

  return 0;
}