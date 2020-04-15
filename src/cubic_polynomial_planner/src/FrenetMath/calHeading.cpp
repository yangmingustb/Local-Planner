//
// Created by ustb on 19-7-8.
//
#include "calHeading.h"
#include <math.h>
#include <vector>

namespace lattice_planner {

double cal_FrenetHeading(lattice_planner::FrenetPose pose0,
                         lattice_planner::FrenetPose pose1) {
  double s0 = pose0.s;
  double rho0 = pose0.rho;
  double s1 = pose1.s;
  double rho1 = pose1.rho;

  double theta = atan2((rho1 - rho0), (s1 - s0));
  return theta;
}
}
