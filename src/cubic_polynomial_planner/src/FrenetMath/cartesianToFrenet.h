//
// Created by ustb on 19-7-12.
//
/**
 * 尽量少地使用迪卡尔坐标转换到Frenet坐标，程序实现效率并不高
 */

#ifndef LATTICEPLANNER_CARTESIANTOFRENET_H
#define LATTICEPLANNER_CARTESIANTOFRENET_H

#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Path.h>
// #include <algorithm>
// #include <functional>
// #include <memory>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include "kdTree.h"
#include "selfType.h"

namespace lattice_planner {

class cartesianToFrenet {
 private:
  std::vector<CubicCoefficients> coefficients_;
  nav_msgs::Path refline_;

  FrenetPose FrtPose_;

 public:
  cartesianToFrenet() = default;
  cartesianToFrenet(nav_msgs::Path &refline,
                    std::vector<CubicCoefficients> &coefficients);

  void setParameters(nav_msgs::Path &refline,
                     std::vector<CubicCoefficients> &coefficients);

  /**
   * the pose of cartesian is transformed into the pose of frenet
   */
  FrenetPose transform(geometry_msgs::PoseWithCovarianceStamped pose);

  /**
   * judge a cartesian pose is the left side or right side of a reference line
   */
  int IsRightOrLeft(point_t p, point_t mappingp_p, point_t mapping_pNeighbour);
};
}

#endif  // LATTICEPLANNER_CARTESIANTOFRENET_H
