//
// Created by ustb on 19-7-12.
//

#include "frenetToCartesian.h"
#include <cmath>
#include <iostream>

namespace lattice_planner {

frenetToCartesian::frenetToCartesian(
    std::vector<CubicCoefficients> &coefficients) {
  coefficients_ = coefficients;
}

void frenetToCartesian::setParameters(
    std::vector<CubicCoefficients> &coefficients) {
  coefficients_ = coefficients;
}

geometry_msgs::PoseStamped frenetToCartesian::transform(FrenetPose frtPose) {
  poseInTheRefLine(frtPose.s);

  // ROS_INFO("poses_of_reference_line() is completed...");
  double x_r = refLinePose_.x;
  double y_r = refLinePose_.y;
  double theta_r = refLinePose_.yaw;

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = x_r + frtPose.rho * cos(theta_r + M_PI / 2.0);
  pose.pose.position.y = y_r + frtPose.rho * sin(theta_r + M_PI / 2.0);
  double theta = theta_r + frtPose.heading;
  geometry_msgs::Quaternion geo_q = tf::createQuaternionMsgFromYaw(theta);
  pose.pose.orientation = geo_q;
  // pose.yaw = theta_r + thetaRho;
  return pose;
}

void frenetToCartesian::poseInTheRefLine(double s) {
  int s_id = 0;
  s_id = binarySearch(s);
  CubicCoefficients param = coefficients_[s_id];
  double s_start = param.s;
  // std::vector<double> a = {{coefficients[s_id][1], coefficients[s_id][2],
  // coefficients[s_id][3], coefficients[s_id][4]}};
  // std::vector<double > b = {{coefficients[s_id][5], coefficients[s_id][6],
  // coefficients[s_id][7], coefficients[s_id][8]}};

  double x = param.a0 + param.a1 * s + param.a2 * s * s + param.a3 * s * s * s;
  double d_x = param.a1 + 2 * param.a2 * s + 3 * param.a3 * s * s;
  double y = param.b0 + param.b1 * s + param.b2 * s * s + param.b3 * s * s * s;
  double d_y = param.b1 + 2 * param.b2 * s + 3 * param.b3 * s * s;

  double theta = std::atan2(d_y, d_x);
  refLinePose_.x = x;
  refLinePose_.y = y;
  refLinePose_.yaw = theta;
}

int frenetToCartesian::binarySearch(double s) {
  int head = 0;
  int tail = coefficients_.size() - 1;
  int mid = 0;

  while (head < tail) {
    mid = (head + tail) / 2;
    if (coefficients_[mid].s < s)
      head = mid + 1;
    else if (s < coefficients_[mid].s)
      tail = mid - 1;
    else
      return mid;
  }

  if (s < coefficients_[head].s)
    return head - 1;
  else
    return head;
}

geometry_msgs::PoseStamped frenetToCartesian::getReflinePose(double s) {
  int s_id = 0;
  s_id = binarySearch(s);
  CubicCoefficients param = coefficients_[s_id];
  double s_start = param.s;
  // std::vector<double> a = {{coefficients[s_id][1], coefficients[s_id][2],
  // coefficients[s_id][3], coefficients[s_id][4]}};
  // std::vector<double > b = {{coefficients[s_id][5], coefficients[s_id][6],
  // coefficients[s_id][7], coefficients[s_id][8]}};

  double x = param.a0 + param.a1 * s + param.a2 * s * s + param.a3 * s * s * s;
  double d_x = param.a1 + 2 * param.a2 * s + 3 * param.a3 * s * s;
  double y = param.b0 + param.b1 * s + param.b2 * s * s + param.b3 * s * s * s;
  double d_y = param.b1 + 2 * param.b2 * s + 3 * param.b3 * s * s;

  double theta = std::atan2(d_y, d_x);
  refLinePose_.x = x;
  refLinePose_.y = y;
  refLinePose_.yaw = theta;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = refLinePose_.x;
  pose.pose.position.y = refLinePose_.y;
  geometry_msgs::Quaternion geo_q =
      tf::createQuaternionMsgFromYaw(refLinePose_.yaw);
  pose.pose.orientation = geo_q;

  return pose;
}

}  // namespace lattice_planner