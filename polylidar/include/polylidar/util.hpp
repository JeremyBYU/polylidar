// I dont want this file, I wanted this inside polylidar/helper.hpp. However delaunator.hpp now relies upon this Matrix<double> class
// and there was a circular dependency. Breaking this out into its own file was the only way for me to fix it.
#ifndef POLYLIDARUTIL
#define POLYLIDARUTIL
#include <cstddef>
#include <limits>
namespace polylidar
{

constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();

constexpr std::size_t operator "" _z ( unsigned long long n )
    { return n; };

template <class T>
class Matrix
{
public:
    T *ptr;
    size_t rows;
    size_t cols;

    Matrix<T>(T *ptr_, size_t rows_, size_t cols_)
        : ptr(ptr_),
          rows(rows_),
          cols(cols_) {}
    Matrix<T>()
        : ptr(nullptr),
          rows(0),
          cols(0) {}

    const T &operator()(size_t i, size_t j) const
    {
        // assert(i >= 0 && i < rows);
        // assert(j >= 0 && j < cols);
        return ptr[i * cols + j];
    }
};
} // namespace polylidar

#endif
