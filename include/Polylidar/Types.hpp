#ifndef POLYLIDAR_TYPES
#define POLYLIDAR_TYPES
#define _USE_MATH_DEFINES
#include <limits>
#include <vector>
#include <cstdint>
#include <array>
#include <unordered_map>

// include a fast parallel hash map implementation
#ifdef PL_USE_STD_UNORDERED_MAP
#include <parallel_hashmap/phmap.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Polylidar {

using VUI = std::vector<size_t>;
using VVUI = std::vector<VUI>;
using Planes = VVUI;
using PlanesGroup = std::vector<Planes>;

// Use Parallel Hash Map unless asked not to
#ifdef PL_USE_STD_UNORDERED_MAP
template <typename T, typename G>
using unordered_map = std::unordered_map<T, G>;
#else
// template <typename T, typename G>
// using unordered_map = phmap::flat_hash_map<T, G>;
template <typename T, typename G>
using unordered_map = std::unordered_map<T, G>;
#endif

using PointHash = unordered_map<size_t, std::vector<size_t>>;
// TODO Change to unordered_set
using EdgeSet = unordered_map<size_t, size_t>;

constexpr std::array<double, 3> PL_DEFAULT_DESIRED_VECTOR{{0, 0, 1}};
constexpr std::array<double, 2> UP_VECTOR = {0.0, 1.0};
constexpr std::array<double, 2> DOWN_VECTOR = {0.0, -1.0};
constexpr std::array<double, 9> PL_DEFAULT_IDENTITY_RM{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
constexpr uint8_t ZERO_UINT8 = static_cast<uint8_t>(0);
constexpr uint8_t ONE_UINT8 = static_cast<uint8_t>(1);
constexpr uint8_t MAX_UINT8 = static_cast<uint8_t>(255);

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
    // Copy Constructor: the ptr can't just copied, check ownership
    Matrix<T>(Matrix<T>& a):own_data(a.own_data), data(a.data), ptr(a.ptr), rows(a.rows), cols(a.cols) {
        if (own_data)
        {
            ptr = data.data();
        }
    }
    Matrix<T>(const Matrix<T>& a): own_data(a.own_data), data(a.data), ptr(a.ptr), rows(a.rows), cols(a.cols) {
        if (own_data)
        {
            ptr = data.data();
        }
    }
    // Move Constructors can be default
    Matrix<T>(Matrix<T>&& other) = default;
    Matrix<T>& operator=(const Matrix<T>& a) = default;
    // Helper function to update ptr to the underlying data structure
    void UpdatePtrFromData() { ptr = data.data(); }
    void UpdatePtrFromData(const size_t rows_, const size_t cols_)
    {
        rows = rows_;
        cols = cols_;
        ptr = data.data();
    }

    const T& operator()(size_t i, size_t j) const
    {
        // assert(i >= 0 && i < rows);
        // assert(j >= 0 && j < cols);
        return ptr[i * cols + j];
    }
    const T& operator()(size_t index) const
    {
        // assert(i >= 0 && i < rows);
        // assert(j >= 0 && j < cols);
        return ptr[index];
    }

    template <class G>
    static Matrix<T> CopyFromDifferentType(G* ptr_from, size_t rows, size_t cols)
    {
        Matrix<T> matrix;
        auto& matrix_data = matrix.data;
        auto total_elements = rows * cols;
        matrix_data.resize(total_elements);
        for (size_t i = 0; i < total_elements; ++i)
        {
            matrix_data[i] = static_cast<T>(ptr_from[i]);
        }
        matrix.UpdatePtrFromData(rows, cols);
        return std::move(matrix);
    }
};

struct Polygon
{
    std::vector<size_t> shell;
    VVUI holes;
    Polygon() : shell(), holes() {}
    VVUI getHoles() const { return holes; }
    void setHoles(VVUI x) { holes = x; }
};

struct ExtremePoint
{
    size_t xr_he = 0;
    size_t xr_pi = 0;
    double xr_val = -1 * std::numeric_limits<double>::infinity();
    size_t xl_he = 0;
    size_t xl_pi = 0;
    double xl_val = std::numeric_limits<double>::infinity();
};

struct PlaneData
{
    std::array<double, 3> plane_normal = PL_DEFAULT_DESIRED_VECTOR;
    std::array<double, 9> rotation_matrix = PL_DEFAULT_IDENTITY_RM;
    bool need_rotation = false;
    uint8_t normal_id = ONE_UINT8;
};

using Polygons = std::vector<Polygon>;
using PolygonsGroup = std::vector<Polygons>;

} // namespace Polylidar

#endif