// I dont want this file, I wanted this inside helper.hpp. However delaunator.hpp now relies upon this Matrix class
// and there was a circular dependency. Breaking this out into its own file was the only way for me to fix it.
#ifndef POLYLIDARUTIL
#define POLYLIDARUTIL
#include <cstddef>
namespace polylidar
{


constexpr std::size_t operator "" _z ( unsigned long long n )
    { return n; };

class Matrix
{
public:
    double *ptr;
    size_t rows;
    size_t cols;

    Matrix(double *ptr_, size_t rows_, size_t cols_)
        : ptr(ptr_),
          rows(rows_),
          cols(cols_) {}
    Matrix()
        : ptr(nullptr),
          rows(0),
          cols(0) {}

    const double &operator()(size_t i, size_t j) const
    {
        // assert(i >= 0 && i < rows);
        // assert(j >= 0 && j < cols);
        return ptr[i * cols + j];
    }
};
} // namespace polylidar

#endif
