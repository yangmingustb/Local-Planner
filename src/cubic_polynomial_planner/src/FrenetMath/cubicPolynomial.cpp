//
// Created by ustb on 19-7-8.
//

#include "cubicPolynomial.h"
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>

using namespace Eigen;

namespace lattice_planner {

CubicPolynomial::CubicPolynomial(const lattice_planner::FrenetPose& start,
                                 const lattice_planner::FrenetPose& end) {
  start_condition_ = start;
  end_condition_ = end;
  double s0 = start_condition_.s;
  double rho0 = start_condition_.rho;
  double theta0 = start_condition_.heading;

  double sg = end_condition_.s;
  double rhog = end_condition_.rho;
  double thetag = end_condition_.heading;

  Matrix4d A;
  Vector4d B(rho0, rhog, tan(theta0), tan(thetag));

  A << 1.0, s0, s0 * s0, s0 * s0 * s0, 1.0, sg, sg * sg, sg * sg * sg, 0.0, 1.0,
      2.0 * s0, 3 * s0 * s0, 0.0, 1.0, 2.0 * sg, 3 * sg * sg;

  Vector4d TmpCoefficients = A.colPivHouseholderQr().solve(B);
  // std::cout<<"print refLine_coefficients:"<<TmpCoefficients<<std::endl;

  cubicCoeffients_.c0 = TmpCoefficients[0];
  cubicCoeffients_.c1 = TmpCoefficients[1];
  cubicCoeffients_.c2 = TmpCoefficients[2];
  cubicCoeffients_.c3 = TmpCoefficients[3];
}

FrenetPath CubicPolynomial::FrenetCubicPolynomial() {
  double step = 0.1;
  double s0 = start_condition_.s;
  double sg = end_condition_.s;
  int length_s = int((sg - s0) / step);

  double coeff0 = cubicCoeffients_.c0;
  double coeff1 = cubicCoeffients_.c1;
  double coeff2 = cubicCoeffients_.c2;
  double coeff3 = cubicCoeffients_.c3;
  double tmp_s = s0;

  for (int i = 0; i < length_s; i++) {
    tmp_s = tmp_s + step;
    double tmp_rho = coeff0 + coeff1 * tmp_s + coeff2 * pow(tmp_s, 2) +
                     coeff3 * pow(tmp_s, 3);
    FrenetPose state0;
    state0.s = tmp_s;
    state0.rho = tmp_rho;

    double next_s = tmp_s + step;
    double next_rho = coeff0 + coeff1 * next_s + coeff2 * pow(next_s, 2) +
                      coeff3 * pow(next_s, 3);
    FrenetPose state1;
    state1.s = next_s;
    state1.rho = next_rho;

    double tmp_theta = cal_FrenetHeading(state0, state1);

    state0.heading = tmp_theta;
    frtPath_.frtPath_.push_back(state0);
  }

  return frtPath_;
}

void CubicPolynomial::print_coefficients() {
  ROS_INFO("print cubic polynomial's coeffis=cients");
  ROS_INFO("%f, %f, %f, %f", cubicCoeffients_.c0, cubicCoeffients_.c1,
           cubicCoeffients_.c2, cubicCoeffients_.c3);
}

cubic_coeffi CubicPolynomial::getCubicCoefficients() {
  return cubicCoeffients_;
}
}  // namespace lattice_planner