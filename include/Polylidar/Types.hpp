#ifndef POLYLIDAR_TYPES
#define POLYLIDAR_TYPES
#define _USE_MATH_DEFINES
#include <limits>
#include <vector>
#include <cstdint>
#include <array>
#include <unordered_map>
#include <deque>
#include <memory>

// include a fast parallel hash map implementation
#ifdef PL_USE_STD_UNORDERED_MAP
#include <parallel_hashmap/phmap.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PL_DEFAULT_ALPHA 0.0
#define PL_DEFAULT_LMAX 1.0
#define PL_DEFAULT_MINTRIANGLES 20
#define PL_DEFAULT_MINHOLEVERTICES 3
#define PL_DEFAULT_MINBBOX 100.0
#define PL_DEFAULT_ZTHRESH 0.0
#define PL_DEFAULT_NORMTHRESH 0.90
#define PL_DEFAULT_NORMTHRESH_MIN 0.90
#define PL_DEFAULT_STRIDE 2
#define PL_DEFAULT_CALC_NORMALS true
#define PL_DEFAULT_TASK_THREADS 4
#define PL_EPS_RADIAN 0.001

namespace Polylidar {

/** @brief Vector of unsigned integers. Often used to represent a set of triangle indices all spatially connected, aka a plane. */
using VUI = std::vector<size_t>;
/** @brief Vector of vector of unsigned integers */
using VVUI = std::vector<VUI>;
/** @brief Each plane (planar triangle set) is a collection of unsigned integers. Planes is just a vector of this. */
using Planes = VVUI;
/** @brief Each group is a collection of planes with the same dominant plane normal. This hold multiple groups. */
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

/** @brief Planes by default should be aligned with the XY Plane */
static constexpr std::array<double, 3> PL_DEFAULT_DESIRED_VECTOR{0, 0, 1};
/** @brief y-axis on XY Plane */
static constexpr std::array<double, 2> UP_VECTOR = {0.0, 1.0};
/** @brief negative y-axis on XY Plane */
static constexpr std::array<double, 2> DOWN_VECTOR = {0.0, -1.0};
/** @brief 3X3 identity rotation matrix*/
static constexpr std::array<double, 9> PL_DEFAULT_IDENTITY_RM{1, 0, 0, 0, 1, 0, 0, 0, 1};
/** @brief Will be used to indicate an invalid triangle */
static constexpr uint8_t ZERO_UINT8 = static_cast<uint8_t>(0);
/** @brief Default valid group */
static constexpr uint8_t ONE_UINT8 = static_cast<uint8_t>(1);
/** @brief Will be used to indicate an invalid triangle */
static constexpr uint8_t MAX_UINT8 = static_cast<uint8_t>(255);

/**
 * This class hold a generic matrix datastructure used to hold much of the data in our meshes (vertices, triangles, etc.).
 * It can own the data or it may not. It can also take ownserhip of a memory buffer.
 * Over time I have become concerned with this datastructure because of this flexibility which can bring bugs if one is not careful.
 * 
 */
template <class T>
class Matrix
{
  public:
    /** @brief Does the matrix own the data pointed to by `ptr`. If yes then ptr == data.data().  */
    bool own_data;
    /** @brief Buffer of memory that may/may-not be used based upon own_data flag  */
    std::vector<T> data;
    /** @brief This is raw pointer that never needs to be freed.
     * It either point to memory in `data` which will be automatically freed during object destruction (`own_data` = true).
     * Or it points some other managed memory controlled by the user (`own_data` = false) 
      */
    T* ptr;
    /** @brief Rows in the matrix  */
    size_t rows;
    /** @brief Columns in the matrix  */
    size_t cols;

    /**
     * @brief Construct a new Matrix< T> object
     * 
     * @param ptr_          Raw pointer to underlying data we DON'T own
     * @param rows_
     * @param cols_ 
     */
    Matrix<T>(T* ptr_, size_t rows_, size_t cols_) : own_data(false), data(), ptr(ptr_), rows(rows_), cols(cols_) {}
    /**
     * @brief Construct a new Matrix< T> object, everything is empty but is owned
     * 
     */
    Matrix<T>() : own_data(true), data(), ptr(data.data()), rows(0), cols(0) {}
    /**
     * @brief Construct a new Matrix< T> object.
     * We will take ownership of this data buffer
     * 
     * @param old_vector    Old vector that will moved and managed by this new Matrix Object
     * @param rows_ 
     * @param cols_ 
     */
    Matrix<T>(std::vector<T>&& old_vector, size_t rows_, size_t cols_)
        : own_data(true), data(), ptr(nullptr), rows(rows_), cols(cols_)
    {
        data = std::move(old_vector);
        ptr = data.data();
    }
    ~Matrix<T>() = default;
    /**
     * @brief Construct a new Matrix< T> object, Copy Constructor.
     * Will take an existing matrix and will perform a copy.
     * If the data is owned it will be copied, if not it wont be copied.
     * @param a 
     */
    Matrix<T>(Matrix<T>& a) : own_data(a.own_data), data(a.data), ptr(a.ptr), rows(a.rows), cols(a.cols)
    {
        if (own_data)
        {
            ptr = data.data();
        }
    }
    /**
     * @brief Construct a new Matrix< T> object, Copy Constructor.
     * Will take an existing matrix and will perform a copy.
     * If the data is owned it will be copied, if not it wont be copied.
     * @param a
     */
    Matrix<T>(const Matrix<T>& a) : own_data(a.own_data), data(a.data), ptr(a.ptr), rows(a.rows), cols(a.cols)
    {
        if (own_data)
        {
            ptr = data.data();
        }
    }
    /**
     * @brief Default move constructor, take ownership of all the data of `other`
     * 
     * @param other 
     */
    Matrix<T>(Matrix<T>&& other) = default;

    /**
     * @brief Copy assignment operator
     * This one is a little tricky. We make a copy of every element in the data.
     * However if we own the data we reassign `ptr` to belong to our new copied buffer 
     * @param a 
     * @return Matrix<T>& 
     */
    Matrix<T>& operator=(const Matrix<T>& a)
    {
        own_data = a.own_data;
        data = a.data;
        ptr = a.ptr;
        rows = a.rows;
        cols = a.cols;
        if (a.own_data)
        {
            ptr = data.data();
        }
        return *this;
    };

    /**
     * @brief Simple helper function to change our `ptr` to point to our own buffer
     * 
     */
    void UpdatePtrFromData() { ptr = data.data(); }
    /**
     * @brief Simply updates our rows and columns and data
     * 
     * @param rows_ 
     * @param cols_ 
     */
    void UpdatePtrFromData(const size_t rows_, const size_t cols_)
    {
        rows = rows_;
        cols = cols_;
        ptr = data.data();
        own_data = true;
    }

    /**
     * @brief Acces an element of the Matrix using 2D indices
     * 
     * @param i 
     * @param j 
     * @return const T& 
     */
    const T& operator()(size_t i, size_t j) const
    {
        // assert(i >= 0 && i < rows);
        // assert(j >= 0 && j < cols);
        return ptr[i * cols + j];
    }
    /**
     * @brief Access an element in the matrix using its underlying buffer 1D index
     * 
     * @param index 
     * @return const T& 
     */
    const T& operator()(size_t index) const
    {
        // assert(i >= 0 && i < rows);
        // assert(j >= 0 && j < cols);
        return ptr[index];
    }

    /**
     * @brief This actually performs a memory copy from an unknown buffer of one type (G)
     * to *our* buffer of a different type (T). We own this new copied data.
     * 
     * @tparam G 
     * @param ptr_from 
     * @param rows 
     * @param cols 
     * @return Matrix<T> 
     */
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

/**
 * Contains the linear rings of a polygon
 * 
 */
struct Polygon
{
    /** @brief Linear ring of our exterior shell. The elements are point indices into a point cloud*/
    std::vector<size_t> shell;
    /** @brief Set of linear rings of interior holes in our polygon*/
    VVUI holes;
    Polygon() : shell(), holes() {}
    VVUI getHoles() const { return holes; }
    void setHoles(VVUI x) { holes = x; }
};

/**
 * Represents point on the far right (x-axis) of hull of a mesh
 * TODO remove the far left x-axis information that I dont use/need
 * 
 */
struct ExtremePoint
{
    size_t xr_he = 0;
    size_t xr_pi = 0;
    double xr_val = -1 * std::numeric_limits<double>::infinity();
    size_t xl_he = 0;
    size_t xl_pi = 0;
    double xl_val = std::numeric_limits<double>::infinity();
};

/**
 * Represents information about a dominant plane normal
 * Contains the surface unit normal and rotation matrix to align this normal to [0,0,1]
 * Needs rotation is false if plane normal is already close to [0,0,1]
 * Each dominant plane normal also has a unique id to represent it
 */
struct PlaneData
{
    std::array<double, 3> plane_normal = PL_DEFAULT_DESIRED_VECTOR;
    std::array<double, 9> rotation_matrix = PL_DEFAULT_IDENTITY_RM;
    bool need_rotation = false;
    uint8_t normal_id = ONE_UINT8;
};
/** @brief Vector of polygons */
using Polygons = std::vector<Polygon>;
/** @brief A group holds polygons for one dominant plane normal. This holds multiple groups. */
using PolygonsGroup = std::vector<Polygons>;

} // namespace Polylidar

#endif