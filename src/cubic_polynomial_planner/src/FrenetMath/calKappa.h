//
// Created by ustb on 19-7-9.
//

#ifndef LATTICEPLANNER_CALKAPPA_H
#define LATTICEPLANNER_CALKAPPA_H

#include "cubicPolynomial.h"
#include "frenetToCartesian.h"
#include "selfType.h"

namespace lattice_planner {

/**
 * calculate the average curvature of a cubic polynomial path
 * @param node
 * @param next_node
 * @return
 */
double trajectory_kappa(const Node node, const Node next_node,
                        std::vector<CubicCoefficients>& coefficients);
}

#endif  // LATTICEPLANNER_CALKAPPA_H
