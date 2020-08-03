#ifndef FASTEXP_H
#define FASTEXP_H

#include "math.h"
#include <cstdint>
#include <cstddef>
#include <vector>
#include "FastExp/product.h"
#include "FastExp/ieee.h"

namespace fastexp
{

enum class Approximation {IEEE, PRODUCT};

/** \brief Fast approximate exponential.
 *
 * This function implements a fast, vectorizable approximation
 * of the exponential function based on the following two articles:
 *
 * - Malossi, A. Cristiano I. & Ineichen, Yves & Bekas, Costas & Curioni,
 *   Alessandro. "Fast Exponential Computation on SIMD Architectures." (2015)
 *   10.13140/2.1.4362.3207.
 * - IEEE, Nicol N. "A fast, compact approximation of the exponential
 *   function." Neural Computation 11.4 (1999): 853-862.
 *
 * The approximation interpolates linearly between points on the curve of
 * the exponential function that can be expressed as 2^i where i is an
 * a signed integer. So yes, that is very approximate ...
 *
 * \tparam Real The floating point type of the arguments.
 * \param x The argument of the exponential function.
 * \return The approximated value of the exponential function.
 */
#ifndef _WIN32
#pragma omp declare simd notinbranch
#endif
template
<
    typename Real,
    template<typename, size_t> class Approximation = IEEE,
    size_t degree = 2
>
inline Real exp(const Real &x)
{
    return Approximation<Real, degree>::evaluate(x);
}

/** \brief Fast approximate array exponential.
 *
 * Applies the fast exponential to an array of given length making
 * use of SIMD instructions if available. To enable vectorization
 * the code needs to be compiled with OpenMP support.
 *
 * \tparam Real The floating point type of the arguments.
 * \param x The array to which apply the exponential function.
 * \return n The number of elements in the array.
 */
template
<
typename Real,
template<typename, size_t> class Approximation = IEEE,
size_t degree = 2
>
inline void exp(Real *x, size_t n) {
    // Vectorized part.
    #pragma omp simd
    for (size_t i = 0; i < n; ++i) {
        Real e = fastexp::exp<Real, Approximation, degree>(x[i]);
        x[i] = e;
    }
}

template
<
typename Real,
template<typename, size_t> class Approximation = IEEE,
size_t degree = 2
>
inline void exp(std::vector<Real> x) {
    // Vectorized part.
    size_t n = x.size();
    Real * x_ptr = &x[0];

    #pragma omp simd
    for (size_t i = 0; i < n; ++i) {
        Real e = fastexp::exp<Real, Approximation, degree>(x_ptr[i]);
        x_ptr[i] = e;
    }
}

}      // fastexp
#endif // FASTEXP_H
