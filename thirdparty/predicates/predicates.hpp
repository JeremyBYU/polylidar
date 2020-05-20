
#ifndef PREDICATES_HPP
#define PREDICATES_HPP


#include <cmath>

#include "predicates.h"

namespace predicates {

template <typename Predicate, typename Continuation>
void perturb2d(
  const Predicate& predicate,
  const double * x0,
  const size_t nx,
  const size_t ny,
  Continuation& continuation
)
{
  double x[] = {x0[0], x0[1]};

  for (size_t i = 0; i < ny; ++i) {
    x[0] = x0[0];

    for (size_t j = 0; j < nx; ++j) {
      const double p = predicate(x);
      continuation(p, x, i, j);

      x[0] = nextafter(x[0], x[0] + 1);
    }
    x[1] = nextafter(x[1], x[1] + 1);
  }
}


} // end of namespace predicates


#endif
