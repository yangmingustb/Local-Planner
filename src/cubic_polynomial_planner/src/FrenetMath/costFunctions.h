//
// Created by ustb on 19-7-8.
//

#ifndef LATTICEPLANNER_COSTFUNCTIONS_H
#define LATTICEPLANNER_COSTFUNCTIONS_H

#include "cubicPolynomial.h"
#include "frenetToCartesian.h"
#include "selfType.h"

namespace lattice_planner {

class costFunction {
 private:
  /* data */
  std::vector<CubicCoefficients> coefficients_;
  std::vector<FrenetPose> obstacles_;
  double reflineRhoValue_;

  Node start_;
  Node end_;
  FrenetPose frtStart_;
  FrenetPose frtEnd_;
  frenetToCartesian frtToCrt_;

  FrenetPath frtPath_;

 public:
  costFunction() = default;

  costFunction(std::vector<CubicCoefficients> &coefficients);

  void setParameters(std::vector<CubicCoefficients> &coefficients,
                     double refline, std::vector<FrenetPose> obstacle,
                     frenetToCartesian &frtToCrt);

  double total_cost(Node start, Node end);

  /**
   * calculate the average curvature of a motion primitive
   * @param node
   * @param next_node
   * @return
   */
  double kappa_cost();

  /**
   * calculate the reference line cost introduced by the lateral offset from the
   * reference line
   * @param start_node
   * @param next_node
   * @param refline
   * @return
   */
  double reference_line_cost();

  /**
   * calculate the collision risk,
   * @param start_node
   * @param next_node
   * @param obstacle
   * @return
   */
  double collision_risk();

  /**
   * calculate the average curvature of a cubic polynomial path
   * @param node
   * @param next_node
   * @return
   */
  double trajectory_kappa();
};

}  // namespace lattice_planner

#endif  // LATTICEPLANNER_COSTFUNCTIONS_H
