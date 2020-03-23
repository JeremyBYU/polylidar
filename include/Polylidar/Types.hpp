#ifndef POLYLIDAR_TYPES
#define POLYLIDAR_TYPES

#include <limits>
#include <vector>

namespace Polylidar {

using VUI = std::vector<size_t>;
using VVUI = std::vector<VUI>;

template <class T>
class Matrix
{
  public:
    const bool own_data;
    std::vector<T> data;
    T* ptr; // This raw pointer never needs to be freed
    size_t rows;
    size_t cols;

    Matrix<T>(T* ptr_, size_t rows_, size_t cols_) : own_data(false), data(), ptr(ptr_), rows(rows_), cols(cols_) {}
    Matrix<T>() : own_data(true), data(), ptr(data.data()), rows(0), cols(0) {}
    Matrix<T>(std::vector<T>&& old_vector, size_t rows_, size_t cols_)
        : own_data(true), data(), ptr(nullptr), rows(rows_), cols(cols_)
    {
        data = std::move(old_vector);
        ptr = data.data();
    }
    ~Matrix<T>() = default;
    Matrix<T>(Matrix<T>& a) = default;
    Matrix<T>(Matrix<T>&& other) = default; // move constructor
    Matrix<T>& operator=(const Matrix<T>& a) = default;

    void UpdatePtrFromData() { ptr = data.data(); }

    const T& operator()(size_t i, size_t j) const
    {
        // assert(i >= 0 && i < rows);
        // assert(j >= 0 && j < cols);
        return ptr[i * cols + j];
    }
};

} // namespace Polylidar

#endif