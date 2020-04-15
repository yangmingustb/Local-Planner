//
// Created by ustb on 19-7-12.
//

#ifndef LATTICEPLANNER_FRENETTOCARTESIAN_H
#define LATTICEPLANNER_FRENETTOCARTESIAN_H

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <array>
#include <vector>
#include "selfType.h"

namespace lattice_planner {

class frenetToCartesian {
 private:
  /* data */
  std::vector<CubicCoefficients> coefficients_;
  FrenetPose frtPose_;
  CartesianPose crtPose_;

  CartesianPose refLinePose_;

 public:
  frenetToCartesian() = default;
  frenetToCartesian(std::vector<CubicCoefficients> &coefficients);

  void setParameters(std::vector<CubicCoefficients> &coefficients);

  /**
   * the frenet pose is transformed into the cartesian pose
   */
  geometry_msgs::PoseStamped transform(FrenetPose frtPose);

  /**
   * input:s,output:x,y,theta
   * calculate the poses of points in the reference line
   * @param s
   * @param coefficients
   * @return
   */
  void poseInTheRefLine(double s);

  /**
   * algorithm:binary search
   * @param coefficients
   * @param s
   * @return
   */
  int binarySearch(double s);

  geometry_msgs::PoseStamped getReflinePose(double s);
};

}  // namespace lattice_planner

#endif  // LATTICEPLANNER_FRENETTOCARTESIAN_H
