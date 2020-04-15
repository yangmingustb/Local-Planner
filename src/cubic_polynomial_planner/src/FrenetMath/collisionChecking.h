/**
 *
 * 1
 * 对于碰撞检查，其中一种方式是基于代价地图，车辆的路径是栅格的，障碍物也是栅格的，
 * 这样对整个路径的栅格与地图中的栅格进行简单的运算即可作出快速地碰撞检查。
 * -------------------------
 * 2
 * 对于不依赖代价地图的碰撞检查，
 * 库：FCL，用于moveit
 * 库：apollo
 *
 * 对于本文，使用三层圆进行检查，算法如下：
 *
 * if(外接圆无碰撞) return no collision
 * else {
 *      if(内接圆有碰撞) return collision
 *      else{
 *          计算包络圆是否碰撞
 *          if(碰撞) return collison;
 *          else return nocollison;
 *      }
 *
 *
 * }
 *
 * 对于碰撞的具体计算方式，本文使用
 *
 */
#ifndef LATTICE_PLANNER_COLLISIONCHECKING_H
#define LATTICE_PLANNER_COLLISIONCHECKING_H

#include <vector>
#include "cartesianToFrenet.h"
#include "selfType.h"

namespace lattice_planner {

/**
 * a vehicle body is overlaid by three little disks and one big disk.
 */
class disksTrajectory {
 private:
  std::vector<VehicleDisks> disksTrajectory_;

  // /**
  //  * 后轴中点
  //  */
  // CartesianPose RearAxleMidPoint_;

  /**
   * vehicle body width
   */
  double VehicleWidth_;

  /**
   * vehicle body length;
   */
  double VehicleLength_;

  /**
   * distance between disks
   */
  double Distance_;

  /**
   * the distancle between the rear axle mid point and the rear boudary of a
   * vehicle body
   */
  double H_;

  // std::vector<CubicCoefficients> coefficients_;
  // nav_msgs::Path refline_;

  cartesianToFrenet crtToFrt_;

  CartesianPath crtPath_;

 public:
  disksTrajectory(
      // std::vector<CubicCoefficients> &coefficients,
      // nav_msgs::Path &refline,
      cartesianToFrenet &crtToFrt, CartesianPath &crtPath);
  // disksTrajectory(CartesianPose &rearAxleMidPoint,
  //             double vehicleWidth,
  //             double vehicleLength,
  //             double H);

  disksTrajectory() = default;

  void setParameters(
      // std::vector<CubicCoefficients> &coefficients,
      // nav_msgs::Path &refline,
      cartesianToFrenet &crtToFrt, CartesianPath &crtPath);

  VehicleDisks generateDisks(CartesianPose &rearAxleMidPoint_);

  std::vector<VehicleDisks> generateDisksTrajectory();

  /**
   * debug function
   */
  void showClassParameters();
};

class CollisionChecker {
 private:
  // disksTrajectory disks_;

  CartesianPath ctsPath_;
  complexPath cpxpath_;
  PointsObstacle obstacles_;
  disksTrajectory disksTrajectoryObj_;

  double rhoLeft_;
  double rhoRight_;

  nav_msgs::Path refline_;
  std::vector<CubicCoefficients> coefficients_;

  cartesianToFrenet crtToFrt_;

 public:
  CollisionChecker(CartesianPath &ctsPath, PointsObstacle &obstacles,
                   double rhoLeft, double rhoRight,
                   //  nav_msgs::Path &refline,
                   //  std::vector<CubicCoefficients> &coefficients,
                   cartesianToFrenet &crtToFrt);

 public:
  /**
   * if collision, return false
   * if no collision, return true
   */
  bool performChecking();
};
}

#endif