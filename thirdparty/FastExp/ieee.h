#ifndef FASTEXP_IEEE_H
#define FASTEXP_IEEE_H

#include "math.h"
#include <cstdint>

namespace fastexp
{
    template<typename Real, size_t degree, size_t i = 0> struct PolynomialFit;
    template<typename Real> struct Info;

    template<typename Real, size_t degree>
    struct IEEE {
        static Real evaluate(Real x) {
            using unsigned_t = typename Info<Real>::unsigned_t;
            constexpr unsigned_t shift = static_cast<unsigned_t>(1) << Info<Real>::shift;

            x *= Info<Real>::log2e;
            Real xi = floor(x);
            Real xf = x - xi;

            Real k = PolynomialFit<Real, degree, 0>::evaluate(xf) + 1.0;
            unsigned_t e = reinterpret_cast<const unsigned_t &>(k);
            e += shift * static_cast<unsigned_t>(xi);
            return reinterpret_cast<Real &>(e);
        }
    };

    ////////////////////////////////////////////////////////////////////////////////
    // Polynomial coefficients for error function fit.
    ////////////////////////////////////////////////////////////////////////////////


    template<> struct Info<float> {
        using unsigned_t = uint32_t;
        static constexpr uint32_t shift = 23;
        static constexpr float  log2e = 1.442695040;
    };

    template<> struct Info<double> {
        using unsigned_t = uint64_t;
        static constexpr uint64_t shift = 52;
        static constexpr double log2e = 1.442695040;
    };

    template<typename Real, size_t degree>
        struct Data;

    template<typename Real>
    struct Data<Real, 1> {
        static constexpr Real coefficients[2] = {-0.05288671,
                                                 0.99232129};
    };
    template<typename Real> constexpr Real Data<Real, 1>::coefficients[2];

    template<typename Real>
    struct Data<Real, 2> {
        static constexpr Real coefficients[3] = {0.00365539,
                                                 0.64960693,
                                                 0.34271434};
    };
    template<typename Real> constexpr Real Data<Real, 2>::coefficients[3];

    template<typename Real>
    struct Data<Real, 3> {
        static constexpr Real coefficients[4] = {-1.77187919e-04,
                                                6.96787180e-01,
                                                2.24169036e-01,
                                                7.90302044e-02};
    };
    template<typename Real> constexpr Real Data<Real, 3>::coefficients[4];

    template<typename Real>
    struct Data<Real, 4> {
        static constexpr Real coefficients[5] = { 6.58721338e-06,
                                                  6.92937406e-01,
                                                  2.41696769e-01,
                                                  5.16742848e-02,
                                                  1.36779598e-02};
    };
    template<typename Real> constexpr Real Data<Real, 4>::coefficients[5];

    template<typename Real>
        struct Data<Real, 5> {
        static constexpr Real coefficients[6] = {6.58721338e-06,
                                                 6.92937406e-01,
                                                 2.41696769e-01,
                                                 5.16742848e-02,
                                                 1.36779598e-02};
    };
    template<typename Real> constexpr Real Data<Real, 5>::coefficients[6];

    template<typename Real>
        struct Data<Real, 6> {
        static constexpr Real coefficients[7] = {-1.97880719e-07,
                                                 6.93156327e-01,
                                                 2.40133447e-01,
                                                 5.58740717e-02,
                                                 8.94160147e-03,
                                                 1.89454334e-03};
    };
    template<typename Real> constexpr Real Data<Real, 6>::coefficients[7];

    template<typename Real>
        struct Data<Real, 7> {
        static constexpr Real coefficients[8] = {4.97074799e-09,
                                                 6.93146861e-01,
                                                 2.40230956e-01,
                                                 5.54792541e-02,
                                                 9.68583180e-03,
                                                 1.23835751e-03,
                                                 2.18728611e-04};
    };
    template<typename Real> constexpr Real Data<Real, 7>::coefficients[8];

    template<typename Real>
        struct Data<Real, 8> {
        static constexpr Real coefficients[9] = {-1.06974751e-10,
                                                    6.93147190e-01,
                                                    2.40226337e-01,
                                                    5.55053726e-02,
                                                    9.61338873e-03,
                                                    1.34310382e-03,
                                                    1.42959529e-04,
                                                    2.16483090e-05};
    };
    template<typename Real> constexpr Real Data<Real, 8>::coefficients[9];

    template<typename Real>
        struct Data<Real, 9> {
        static constexpr Real coefficients[10] = {2.00811867e-12,
                                                    6.93147180e-01,
                                                    2.40226512e-01,
                                                    5.55040573e-02,
                                                    9.61838113e-03,
                                                    1.33265219e-03,
                                                    1.55193275e-04,
                                                    1.41484217e-05,
                                                    1.87497191e-06};
    };
    template<typename Real> Real constexpr Data<Real, 9>::coefficients[10];

    template<typename Real, size_t degree, size_t i>
        struct PolynomialFit {
            inline static Real evaluate(Real x) {
                Real p = PolynomialFit<Real, degree, i + 1>::evaluate(x) * x;
                p += Data<Real, degree>::coefficients[i];
                return p;
            }
        };

    template<typename Real, size_t degree>
        struct PolynomialFit<Real, degree, degree> {
        inline static Real evaluate(Real x) {
            return Data<Real, degree>::coefficients[degree];
        }
    };

    template<typename Real>
        struct PolynomialFit<Real, 0, 0> {
        inline static Real evaluate(Real x) {
            return x;
        }
    };

}      // fastexp
#endif // FASTEXP_IEEE_H
