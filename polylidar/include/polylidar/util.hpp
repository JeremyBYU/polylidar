// I dont want this file, I wanted this inside polylidar/helper.hpp. However delaunator.hpp now relies upon this Matrix<double> class
// and there was a circular dependency. Breaking this out into its own file was the only way for me to fix it.
#ifndef POLYLIDARUTIL
#define POLYLIDARUTIL
#include <cstddef>
#include <limits>
#include <chrono>
#include <iostream>
namespace polylidar
{

constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();
constexpr std::size_t operator"" _z(unsigned long long n) { return n; }

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

class Timer
{
    typedef std::chrono::high_resolution_clock high_resolution_clock;
    typedef std::chrono::microseconds microseconds;

  public:
    explicit Timer(bool run = false):_start()
    {
        if (run) Reset();
    }
    void Reset() { _start = high_resolution_clock::now(); }
    microseconds Elapsed() const
    {
        return std::chrono::duration_cast<microseconds>(high_resolution_clock::now() - _start);
    }
    template <typename T, typename Traits>
    friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out, const Timer& timer)
    {
        return out << timer.Elapsed().count();
    }

  private:
    high_resolution_clock::time_point _start;
};

} // namespace polylidar

#endif
