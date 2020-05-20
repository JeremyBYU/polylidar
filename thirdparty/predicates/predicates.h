
#ifndef PREDICATES_H
#define PREDICATES_H

#include <stddef.h>

#include "predicatesDLLExport.h"

#ifdef __cplusplus
extern "C"
{
namespace predicates {
#endif

/**
 * Robust, adaptive-precision geometric predicates
 */

PREDICATES_EXPORT
double orient2d(const double * pa, const double * pb, const double * pc);

PREDICATES_EXPORT
double orient3d(const double * pa,
                const double * pb,
                const double * pc,
                const double * pd);

PREDICATES_EXPORT
double incircle(const double * pa,
                const double * pb,
                const double * pc,
                const double * pd);

PREDICATES_EXPORT
double insphere(const double * pa,
                const double * pb,
                const double * pc,
                const double * pd,
                const double * pe);


/**
 * Nonrobust, fast geometric predicates.
 */

PREDICATES_EXPORT
double orient2dfast(const double * pa, const double * pb, const double * pc);

PREDICATES_EXPORT
double orient3dfast(const double * pa,
                    const double * pb,
                    const double * pc,
                    const double * pd);

PREDICATES_EXPORT
double incirclefast(const double * pa,
                    const double * pb,
                    const double * pc,
                    const double * pd);

PREDICATES_EXPORT
double inspherefast(const double * pa,
                    const double * pb,
                    const double * pc,
                    const double * pd,
                    const double * pe);

#ifdef __cplusplus
} // end of namespace predicates

} // end of extern "C"
#endif

#endif
