//
// Created by ustb on 19-7-12.
//

#include "cartesianToFrenet.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>

namespace lattice_planner {

cartesianToFrenet::cartesianToFrenet(
    nav_msgs::Path &refline, std::vector<CubicCoefficients> &coefficients) {
  refline_ = refline;
  coefficients_ = coefficients;
}

void cartesianToFrenet::setParameters(
    nav_msgs::Path &refline, std::vector<CubicCoefficients> &coefficients) {
  refline_ = refline;
  coefficients_ = coefficients;
}

FrenetPose cartesianToFrenet::transform(
    geometry_msgs::PoseWithCovarianceStamped pose) {
  ROS_INFO("cartesian to frenet is begining...");

  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double cartesian_yaw = yaw;
  // double cartesian_yaw=M_PI/2;

  pointVec points;
  point_t pt;

  for (auto i : refline_.poses) {
    pt = {i.pose.position.x, i.pose.position.y};
    points.push_back(pt);
  }
  ROS_INFO("refPath length: %d", refline_.poses.size());
  KDTree tree(points);

  point_t point_;
  pointIndex pointAndId_;  // record the mapping point in the reference line

  point_ = {pose.pose.pose.position.x, pose.pose.pose.position.y};
  // point_ = {10, 50};

  // lattice_planner::point_t nearst_p = tree.nearest_point(point_);
  // ROS_INFO("nearst position: %f, %f", nearst_p[0], nearst_p[1]);

  // record the nearst point in the reference path waypoints
  pointIndex pointAndId = tree.nearest_pointIndex(point_);
  point_t nearst_p2 = pointAndId.first;
  size_t nearstPointID = pointAndId.second;

  // ROS_INFO("nearst position: %f, %f, id: %d", nearst_p2[0], nearst_p2[1],
  // nearstPointID);

  // To calculate the mapping point in the reference line, there are two
  // methods:
  // 1. kd-tree, search the nearst point in the reference line
  // 2. nonlinear optimization, the solvers can be ceres or ipopt

  if (nearstPointID > 0 and nearstPointID < (refline_.poses.size() - 1)) {
    CubicCoefficients currentPointPara = coefficients_[nearstPointID];
    CubicCoefficients lastPointPara = coefficients_[nearstPointID - 1];
    CubicCoefficients nextPointPara = coefficients_[nearstPointID + 1];

    double step = 0.02;  // search step is 0.01[m]
    int n = int((currentPointPara.s - lastPointPara.s) / step);

    pointVec last_points;

    for (int i_n = 0; i_n != n; i_n++) {
      double s = lastPointPara.s + i_n * step;
      double x = lastPointPara.a0 + lastPointPara.a1 * s +
                 lastPointPara.a2 * s * s + lastPointPara.a3 * s * s * s;
      double y = lastPointPara.b0 + lastPointPara.b1 * s +
                 lastPointPara.b2 * s * s + lastPointPara.b3 * s * s * s;
      point_t last_pt = {x, y};
      last_points.push_back(last_pt);
    }

    for (int i_n = 0; i_n != n; i_n++) {
      double s = currentPointPara.s + i_n * step;
      double x = currentPointPara.a0 + currentPointPara.a1 * s +
                 currentPointPara.a2 * s * s + currentPointPara.a3 * s * s * s;
      double y = currentPointPara.b0 + currentPointPara.b1 * s +
                 currentPointPara.b2 * s * s + currentPointPara.b3 * s * s * s;
      point_t last_pt = {x, y};
      last_points.push_back(last_pt);
    }

    KDTree tmp_tree(last_points);

    // lattice_planner::point_t mapping_p = tmp_tree.nearest_point(point_);

    // ROS_INFO("nearst position: %f, %f", mapping_p[0], mapping_p[1]);
    pointAndId_ = tmp_tree.nearest_pointIndex(point_);
    point_t mapping_p2 = pointAndId_.first;
    size_t nearstPointID_ = pointAndId_.second;
    point_t mapping_pNeighbour = last_points[nearstPointID_ + 1];
    int flag = IsRightOrLeft(point_, mapping_p2, mapping_pNeighbour);

    double mapping_distance = sqrt(pow(point_[0] - mapping_p2[0], 2) +
                                   pow(point_[1] - mapping_p2[1], 2));
    double mapping_s = lastPointPara.s + nearstPointID_ * step;
    double mapping_heading;
    double d_x, d_y;
    if (mapping_s < currentPointPara.s) {
      d_x = lastPointPara.a1 + 2 * lastPointPara.a2 * mapping_s +
            3 * lastPointPara.a3 * mapping_s * mapping_s;
      d_y = lastPointPara.b1 + 2 * lastPointPara.b2 * mapping_s +
            3 * lastPointPara.b3 * mapping_s * mapping_s;

    }

    else {
      d_x = currentPointPara.a1 + 2 * currentPointPara.a2 * mapping_s +
            3 * currentPointPara.a3 * mapping_s * mapping_s;
      d_y = currentPointPara.b1 + 2 * currentPointPara.b2 * mapping_s +
            3 * currentPointPara.b3 * mapping_s * mapping_s;
    }

    double theta_refline = std::atan2(d_y, d_x);
    mapping_heading = cartesian_yaw - theta_refline;

    FrtPose_.s = mapping_s;
    FrtPose_.rho = mapping_distance * flag;
    FrtPose_.heading = mapping_heading;

  } else if (nearstPointID < 1) {
    /* code */
    CubicCoefficients currentPointPara = coefficients_[nearstPointID];
    CubicCoefficients nextPointPara = coefficients_[nearstPointID + 1];

    double step = 0.02;  // search step is 0.01[m]
    int n = int((nextPointPara.s - currentPointPara.s) / step);

    pointVec last_points;
    for (int i_n = 0; i_n != n; i_n++) {
      double s = currentPointPara.s + i_n * step;
      double x = currentPointPara.a0 + currentPointPara.a1 * s +
                 currentPointPara.a2 * s * s + currentPointPara.a3 * s * s * s;
      double y = currentPointPara.b0 + currentPointPara.b1 * s +
                 currentPointPara.b2 * s * s + currentPointPara.b3 * s * s * s;
      point_t last_pt = {x, y};
      last_points.push_back(last_pt);
    }

    KDTree tmp_tree(last_points);

    // lattice_planner::point_t mapping_p = tmp_tree.nearest_point(point_);

    // ROS_INFO("nearst position: %f, %f", mapping_p[0], mapping_p[1]);
    pointAndId_ = tmp_tree.nearest_pointIndex(point_);
    point_t mapping_p2 = pointAndId_.first;
    size_t nearstPointID_ = pointAndId_.second;
    point_t mapping_pNeighbour = last_points[nearstPointID_ + 1];
    int flag = IsRightOrLeft(point_, mapping_p2, mapping_pNeighbour);

    double mapping_distance = sqrt(pow(point_[0] - mapping_p2[0], 2) +
                                   pow(point_[1] - mapping_p2[1], 2));
    double mapping_s = currentPointPara.s + nearstPointID_ * step;
    double mapping_heading;
    double d_x, d_y;
    d_x = currentPointPara.a1 + 2 * currentPointPara.a2 * mapping_s +
          3 * currentPointPara.a3 * mapping_s * mapping_s;
    d_y = currentPointPara.b1 + 2 * currentPointPara.b2 * mapping_s +
          3 * currentPointPara.b3 * mapping_s * mapping_s;
    double theta_refline = std::atan2(d_y, d_x);
    mapping_heading = cartesian_yaw - theta_refline;

    FrtPose_.s = mapping_s;
    FrtPose_.rho = mapping_distance * flag;
    FrtPose_.heading = mapping_heading;
  } else {
    CubicCoefficients currentPointPara = coefficients_[nearstPointID];
    CubicCoefficients lastPointPara = coefficients_[nearstPointID - 1];
    double step = 0.02;  // search step is 0.01[m]
    int n = int((currentPointPara.s - lastPointPara.s) / step);

    pointVec last_points;

    for (int i_n = 0; i_n != n; i_n++) {
      double s = lastPointPara.s + i_n * step;
      double x = lastPointPara.a0 + lastPointPara.a1 * s +
                 lastPointPara.a2 * s * s + lastPointPara.a3 * s * s * s;
      double y = lastPointPara.b0 + lastPointPara.b1 * s +
                 lastPointPara.b2 * s * s + lastPointPara.b3 * s * s * s;
      point_t last_pt = {x, y};
      last_points.push_back(last_pt);
    }
    KDTree tmp_tree(last_points);

    // lattice_planner::point_t mapping_p = tmp_tree.nearest_point(point_);

    // ROS_INFO("nearst position: %f, %f", mapping_p[0], mapping_p[1]);
    pointAndId_ = tmp_tree.nearest_pointIndex(point_);
    point_t mapping_p2 = pointAndId_.first;
    size_t nearstPointID_ = pointAndId_.second;

    point_t mapping_pNeighbour = last_points[nearstPointID_ + 1];
    int flag = IsRightOrLeft(point_, mapping_p2, mapping_pNeighbour);

    double mapping_distance = sqrt(pow(point_[0] - mapping_p2[0], 2) +
                                   pow(point_[1] - mapping_p2[1], 2));
    double mapping_s = lastPointPara.s + nearstPointID_ * step;
    double mapping_heading;
    double d_x, d_y;
    d_x = lastPointPara.a1 + 2 * lastPointPara.a2 * mapping_s +
          3 * lastPointPara.a3 * mapping_s * mapping_s;
    d_y = lastPointPara.b1 + 2 * lastPointPara.b2 * mapping_s +
          3 * lastPointPara.b3 * mapping_s * mapping_s;
    double theta_refline = std::atan2(d_y, d_x);
    mapping_heading = cartesian_yaw - theta_refline;

    FrtPose_.s = mapping_s;
    FrtPose_.rho = mapping_distance * flag;
    FrtPose_.heading = mapping_heading;
  }

  // if (nearstPointID >0 ){
  //     CubicCoefficients currentPointPara = coefficients_[nearstPointID];
  //     CubicCoefficients lastPointPara = coefficients_[nearstPointID-1];

  //     double x = currentPointPara.a0 + currentPointPara.a1 * s +
  //     currentPointPara.a2 * s * s + currentPointPara.a3 * s * s * s;
  //     double d_x = currentPointPara.a1 + 2 * currentPointPara.a2 * s + 3 *
  //     currentPointPara.a3 * s * s;
  //     double y = currentPointPara.b0 + currentPointPara.b1 * s +
  //     currentPointPara.b2 * s * s + currentPointPara.b3 * s * s * s;
  //     double d_y = currentPointPara.b1 + 2 * currentPointPara.b2 * s + 3 *
  //     currentPointPara.b3 * s * s;

  //     double theta = std::atan2(d_y, d_x);

  //     geometry_msgs::PoseStamped refLine_pose;
  //     refLine_pose.pose.position.x = x;
  //     refLine_pose.pose.position.y = y;
  //     geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);
  //     refLine_pose.pose.orientation = q;
  //     refLine_pose.header.stamp=ros::Time::now();
  //     // this_pose_stamped.header.frame_id="/my_frame";
  //     refLine_pose.header.frame_id="/map";

  // }
  // else{
  //     CubicCoefficients currentPointPara = coefficients_[nearstPointID];
  // }
  ROS_INFO("start cartesian pose.x:%f,y:%f,yaw:%f", pose.pose.pose.position.x,
           pose.pose.pose.position.y, cartesian_yaw);
  ROS_INFO("the mapping point in the reference line.x:%f,y:%f",
           pointAndId_.first[0], pointAndId_.first[1]);
  ROS_INFO("the start Frenet pose.s:%f, rho:%f, heading: %f", FrtPose_.s,
           FrtPose_.rho, FrtPose_.heading);

  return FrtPose_;
}

int cartesianToFrenet::IsRightOrLeft(point_t p, point_t mappingp_p,
                                     point_t mapping_pNeighbour) {
  point_t a = {mapping_pNeighbour[0] - mappingp_p[0],
               mapping_pNeighbour[1] - mappingp_p[1]};

  point_t b = {p[0] - mappingp_p[0], p[1] - mappingp_p[1]};

  double flag = a[0] * b[1] - a[1] * b[0];
  if (flag > 0.0)
    return 1;
  else if (flag < 0.0)
    return -1;
  else
    return 0;
}

}  // namespace lattice_planner

#ifdef BUILD_INDIVIDUAL
int main() {
  lattice_planner::pointVec points;
  lattice_planner::point_t pt;

  pt = {0.0, 0.0};
  points.push_back(pt);
  pt = {1.0, 0.0};
  points.push_back(pt);
  pt = {0.0, 1.0};
  points.push_back(pt);
  pt = {1.0, 1.0};
  points.push_back(pt);
  pt = {0.5, 0.5};
  points.push_back(pt);

  lattice_planner::KDTree tree(points);

  std::cout << "nearest test\n";
  pt = {0.8, 0.2};
  auto res = tree.nearest_point(pt);
  for (double b : res) {
    std::cout << b << " ";
  }
  std::cout << '\n';

  return 0;
}
#endif