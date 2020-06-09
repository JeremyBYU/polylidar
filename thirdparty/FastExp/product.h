#ifndef FASTEXP_PRODUCT_H
#define FASTEXP_PRODUCT_H

#include "math.h"
#include <cstdint>

namespace fastexp {

template<typename Real, size_t degree, size_t i>
    struct Recursion {
        static Real evaluate(Real x) {
            constexpr Real c = 1.0 / static_cast<Real>(1u << degree);
            x = Recursion<Real, degree, i + 1>::evaluate(x);
            return x * x;
        }
    };

template<typename Real, size_t degree>
    struct Recursion<Real, degree, degree> {
    static Real evaluate(Real x) {
        constexpr Real c = 1.0 / static_cast<Real>(1u << degree);
        x = 1.0 + c * x;
        return x;
    }
};

template<typename Real, size_t degree>
struct Product {
    static Real evaluate(Real x) {
        return Recursion<Real, degree, 0>::evaluate(x);
    }
};


}      // fastexp
#endif // FASTEXP_PRODUCT_H
