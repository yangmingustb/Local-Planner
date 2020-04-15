//
// Created by ustb on 19-7-8.
//

#ifndef LATTICEPLANNER_CUBICPOLYNOMIAL_H
#define LATTICEPLANNER_CUBICPOLYNOMIAL_H

#include <array>
#include <vector>
#include "calHeading.h"
#include "selfType.h"

namespace lattice_planner {

/**
 * a struct:store cubic polynomial's coefficients
 */
struct cubic_coeffi {
  /* data */
  double c0, c1, c2, c3;
};

class CubicPolynomial {
 public:
  CubicPolynomial() = default;

  CubicPolynomial(const FrenetPose& start, const FrenetPose& end);

  /**
   * x0 is the value when f(x = 0);
   * dx0 is the value when f'(x = 0);
   * ddx0 is the value when f''(x = 0);
   * f(x = param) = x1
   */

  FrenetPath FrenetCubicPolynomial();

  cubic_coeffi getCubicCoefficients();

  void print_coefficients();

 private:
  cubic_coeffi cubicCoeffients_;
  FrenetPose start_condition_;
  FrenetPose end_condition_;
  FrenetPath frtPath_;
};
}

#endif  // LATTICEPLANNER_CUBICPOLYNOMIAL_H
