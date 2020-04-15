//
// Created by ustb on 19-7-12.
//

#include "referenceLine.h"
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <istream>

namespace reference_line {

// std::vector<double > waypoints_x={{0.0, 20.0, 50, 100.0, 150.0, 220.0, 300.0,
// 350.0, 400.0, 430.0, 370.0, 300, 200.0}};
// std::vector<double > waypoints_y={{0.0, 70.0, 100, 120.0, 100.0, 150.0,
// 180.0, 150.0, 110.0, 20.0, -80.0, -80.0, -80.0}};

RefLine::RefLine() {
  // geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7;

  // p1.x=0.0;
  // p1.y=0.0;
  // p2.x=20.0;
  // p2.y=70.0;
  // p3.x=50.0;
  // p3.y=100;
  // p4.x=100;
  // p4.y=120;
  // p5.x=150;
  // p5.y=100;
  // p6.x=220;
  // p6.y=150;
  // p7.x=300;
  // p7.y=180;

  // waypoints_.push_back(p1);
  // waypoints_.push_back(p2);
  // waypoints_.push_back(p3);
  // waypoints_.push_back(p4);
  // waypoints_.push_back(p5);
  // waypoints_.push_back(p6);
  // waypoints_.push_back(p7);

  std::vector<double> waypoints_x = {{0.0, 20.0, 50, 100.0, 150.0, 220.0, 300.0,
                                      350.0, 400.0, 430.0, 370.0, 300, 200.0}};
  std::vector<double> waypoints_y = {{0.0, 70.0, 100, 120.0, 120.0, 150.0,
                                      180.0, 150.0, 110.0, 20.0, -80.0, -80.0,
                                      -80.0}};

  std::ofstream writeFile;

  writeFile.open("/tmp/coefficients_test.csv",
                 std::ios::ate);  // 打开模式可省略
  ROS_INFO("the current path is");
  writeFile.close();
  std::vector<double> x = waypoints_x;
  std::vector<double> y = waypoints_y;
  std::vector<double> r_x, r_y, r_heading, r_curvature, r_s;

  cubicSpline::Spline2D cubic_spline(x, y);
  for (double i = 0; i < cubic_spline.s.back(); i = i + 0.1) {
    std::array<double, 2> point_ = cubic_spline.calculatePosition(i);
    r_x.push_back(point_[0]);
    r_y.push_back(point_[1]);
    r_heading.push_back(cubic_spline.calculateHeading(i));
    r_curvature.push_back(cubic_spline.calculateCurvature(i));
    r_s.push_back(i);
  }
  // std::cout<<"----------------cubic spline--------------"<<std::endl;
  // std::cout<<r_x.size()<<" "<<r_y.size()<<" "<<r_heading.size()<<" "
  // <<r_curvature.size()<<" "<< r_s.size()<<std::endl;

  SparseWayPoints(r_x, r_y, r_heading, r_s);
  ROS_INFO("refline initialing... coefficients size: %d", coefficients_.size());
}

void RefLine::SparseWayPoints(std::vector<double> r_x, std::vector<double> r_y,
                              std::vector<double> r_heading,
                              std::vector<double> r_s) {
  std::ofstream writeFile;
  writeFile.open("/tmp/coefficients_test.csv",
                 std::ios::app);  // 打开模式可省略
  int step = 20;
  for (auto i = 0; i < r_x.size(); i = i + step) {
    // append the last point to the coefficient_list
    if ((i + step) > (r_x.size() - 1)) {
      std::vector<double> pose = {{r_x[i], r_y[i], r_heading[i]}};
      std::vector<double> next_pose = {
          {r_x.back(), r_y.back(), r_heading.back()}};
      double s0 = r_s[i];
      double sf = r_s.back();

      arcLengthRefLine(pose, next_pose, s0, sf, writeFile);

    }

    else {
      std::vector<double> pose = {{r_x[i], r_y[i], r_heading[i]}};
      std::vector<double> next_pose = {
          {r_x[i + step], r_y[i + step], r_heading[i + step]}};
      double s0 = r_s[i];
      double sf = r_s[i + step];

      arcLengthRefLine(pose, next_pose, s0, sf, writeFile);
    }
    // std::cout<<"---------------tmp coefficients------"<<std::endl;
    // std::cout<<TmpCoefficients.size()<<std::endl;
  }
}

void RefLine::arcLengthRefLine(std::vector<double> pose,
                               std::vector<double> nextPose, double s0,
                               double sf, std::ofstream &writeFile) {
  // std::ofstream writeFile;
  // writeFile.open("/home/ustb/coefficients_test.csv", std::ios::app); //
  // 打开模式可省略
  double x0 = pose[0];
  double y0 = pose[1];
  double theta0 = pose[2];
  double xf = nextPose[0];
  double yf = nextPose[1];
  double thetaf = nextPose[2];

  Eigen::Matrix4d A;
  A << 1.0, s0, s0 * s0, s0 * s0 * s0, 1.0, sf, sf * sf, sf * sf * sf, 0.0, 1.0,
      2.0 * s0, 3 * s0 * s0, 0.0, 1.0, 2.0 * sf, 3 * sf * sf;
  Eigen::Vector4d B(x0, xf, cos(theta0), cos(thetaf));
  Eigen::Vector4d a_vec = A.colPivHouseholderQr().solve(B);

  Eigen::Vector4d C(y0, yf, sin(theta0), sin(thetaf));
  Eigen::Vector4d b_vec = A.colPivHouseholderQr().solve(C);

  arc_length_parameter p;
  p.s = s0;
  p.a0 = a_vec[0];
  p.a1 = a_vec[1];
  p.a2 = a_vec[2];
  p.a3 = a_vec[3];
  p.b0 = b_vec[0];
  p.b1 = b_vec[1];
  p.b2 = b_vec[2];
  p.b3 = b_vec[3];

  writeFile << std::setiosflags(std::ios::fixed) << std::setprecision(10) << s0
            << ',' << std::setiosflags(std::ios::fixed) << std::setprecision(10)
            << p.a0 << ',' << std::setiosflags(std::ios::fixed)
            << std::setprecision(10) << p.a1 << ','
            << std::setiosflags(std::ios::fixed) << std::setprecision(10)
            << p.a2 << ',' << std::setiosflags(std::ios::fixed)
            << std::setprecision(10) << p.a3 << ','
            << std::setiosflags(std::ios::fixed) << std::setprecision(10)
            << p.b0 << ',' << std::setiosflags(std::ios::fixed)
            << std::setprecision(10) << p.b1 << ','
            << std::setiosflags(std::ios::fixed) << std::setprecision(10)
            << p.b2 << ',' << std::setiosflags(std::ios::fixed)
            << std::setprecision(10) << p.b3 << std::endl;

  coefficients_.push_back(p);
}

std::vector<arc_length_parameter> RefLine::ref_coefficients_output() const {
  return coefficients_;
}

nav_msgs::Path RefLine::generateRefLine_inRviz() {
  refline_waypoints_.header.stamp = ros::Time::now();
  // path.header.frame_id="/my_frame";
  refline_waypoints_.header.frame_id = "/map";

  for (arc_length_parameter i : coefficients_) {
    geometry_msgs::PoseStamped Refline_pose = poses_of_reference_line(i.s);

    refline_waypoints_.poses.push_back(Refline_pose);
  }

  return refline_waypoints_;
}

geometry_msgs::PoseStamped RefLine::poses_of_reference_line(double s) {
  int s_id = 0;
  s_id = binary_search(s);
  arc_length_parameter param = coefficients_[s_id];

  // std::cout<<"id:"<<s_id<<" "<<"s:"<<param.s<<" "<<"a0:"<<param.a0<<"
  // "<<"a1:"<<param.a1
  // <<"a2:"<<param.a2<<" "<<"a3:"<<param.a3<<" "<<"b0:"<<param.b0<<"
  // "<<"b1:"<<param.b1<<" "
  // <<"b2:"<<param.b2<<" "<<"b3:"<<param.b3<<" "<<std::endl;
  // double s_start = param.s;
  // std::vector<double> a = {{coefficients[s_id][1], coefficients[s_id][2],
  // coefficients[s_id][3], coefficients[s_id][4]}};
  // std::vector<double > b = {{coefficients[s_id][5], coefficients[s_id][6],
  // coefficients[s_id][7], coefficients[s_id][8]}};

  double x = param.a0 + param.a1 * s + param.a2 * s * s + param.a3 * s * s * s;
  double d_x = param.a1 + 2 * param.a2 * s + 3 * param.a3 * s * s;
  double y = param.b0 + param.b1 * s + param.b2 * s * s + param.b3 * s * s * s;
  double d_y = param.b1 + 2 * param.b2 * s + 3 * param.b3 * s * s;

  double theta = std::atan2(d_y, d_x);

  geometry_msgs::PoseStamped refLine_pose;
  refLine_pose.pose.position.x = x;
  refLine_pose.pose.position.y = y;
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);
  refLine_pose.pose.orientation = q;
  refLine_pose.header.stamp = ros::Time::now();
  // this_pose_stamped.header.frame_id="/my_frame";
  refLine_pose.header.frame_id = "/map";
  return refLine_pose;
}

int RefLine::binary_search(double s) {
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

nav_msgs::Path RefLine::readCoefficientsFromFile() {
  std::vector<arc_length_parameter> coeff = coefficients_;
  nav_msgs::Path refline_waypointsTest = refline_waypoints_;
  coefficients_.clear();
  refline_waypoints_.poses.clear();
  std::ifstream readFile("/tmp/coefficients_test.csv");
  std::string lineStr;
  while (getline(readFile, lineStr)) {
    // 存成二维表结构
    std::stringstream ss(lineStr);
    std::string str_data;
    std::vector<double> lineArray;
    // 按照逗号分隔
    while (getline(ss, str_data, ',')) {
      double data = std::atof(str_data.c_str());
      lineArray.push_back(data);
    }

    arc_length_parameter p2;
    p2.s = lineArray[0];
    p2.a0 = lineArray[1];
    p2.a1 = lineArray[2];
    p2.a2 = lineArray[3];
    p2.a3 = lineArray[4];
    p2.b0 = lineArray[5];
    p2.b1 = lineArray[6];
    p2.b2 = lineArray[7];
    p2.b3 = lineArray[8];
    coefficients_.push_back(p2);
  }

  ROS_INFO("coeffcients size: %d", coeff.size());
  ROS_INFO("coefficients2 size: %d", coefficients_.size());

  nav_msgs::Path refPath = generateRefLine_inRviz();
  bool b = isSameData(coeff, refline_waypointsTest);
  // std::cout<< "is same data:"<<b<<std::endl;
  ROS_INFO("coeff is same with coefficients_:%d", b);

  return refPath;
}

bool RefLine::isSameData(std::vector<arc_length_parameter> &coeff,
                         nav_msgs::Path &path) {
  // calculate coeffcients error
  double maxCoeffError = 0;
  int MaxCoeffErrId = 0;
  int j;
  for (j = 0; j != coefficients_.size(); j++) {
    double error_s = coefficients_[j].s - coeff[j].s;
    double error_a0 = coefficients_[j].a0 - coeff[j].a0;
    double error_a1 = coefficients_[j].a1 - coeff[j].a1;
    double error_a2 = coefficients_[j].a2 - coeff[j].a2;
    double error_a3 = coefficients_[j].a3 - coeff[j].a3;
    double error_b0 = coefficients_[j].b0 - coeff[j].b0;
    double error_b1 = coefficients_[j].b1 - coeff[j].b1;
    double error_b2 = coefficients_[j].b2 - coeff[j].b2;
    double error_b3 = coefficients_[j].b3 - coeff[j].b3;

    double sum = abs(error_s) + abs(error_a0) + abs(error_a1) + abs(error_a2) +
                 abs(error_a3) + abs(error_b0) + abs(error_b1) + abs(error_b2) +
                 abs(error_b3);

    if (abs(sum) >= 0.0001) {
      ROS_INFO("COEFFICIENTS ERROR GENERATE POSITION: %d", j);
      ROS_INFO("COEFFICIENTS ERROR GENERATE value: %f", sum);
      if (maxCoeffError < abs(sum)) {
        maxCoeffError = abs(sum);
        MaxCoeffErrId = j;
      }
      // return false;
    }
  }

  /**
   * calculate pose error
  */
  double MaxPoseErr = 0;
  int MaxPoseErrId = 0;
  int q;
  for (q = 0; q != refline_waypoints_.poses.size(); q++) {
    double dx = path.poses[q].pose.position.x -
                refline_waypoints_.poses[q].pose.position.x;
    double dy = path.poses[q].pose.position.y -
                refline_waypoints_.poses[q].pose.position.y;
    double z = sqrt(pow(dx, 2) + pow(dy, 2));

    if (z > 0.005) {
      /* code */
      ROS_INFO("generate POSE error position: %d", q);
      ROS_INFO("generate POSE error value: %f", z);
      // return false;
      if (MaxPoseErr < z) {
        MaxPoseErr = z;
        MaxPoseErrId = q;
      }
    }
  }
  ROS_INFO("MAX COEFFICIENTS ERROR ID: %d", MaxCoeffErrId);
  ROS_INFO("MAX COEFFICIENTS ERROR: %f", maxCoeffError);
  ROS_INFO("MAX POSE ERROR ID: %d", MaxPoseErrId);
  ROS_INFO("MAX POSE ERROR: %f", MaxPoseErr);

  ROS_INFO("------------------");
  ROS_INFO("csv coefficients: %f, %f, %f, %f ", coefficients_[MaxCoeffErrId].a0,
           coefficients_[MaxCoeffErrId].a1, coefficients_[MaxCoeffErrId].a2,
           coefficients_[MaxCoeffErrId].a3);
  ROS_INFO("initial coeff: %f, %f, %f, %f ", coeff[MaxCoeffErrId].a0,
           coeff[MaxCoeffErrId].a1, coeff[MaxCoeffErrId].a2,
           coeff[MaxCoeffErrId].a3);
  ROS_INFO("csv position: %f, %f",
           refline_waypoints_.poses[MaxPoseErrId].pose.position.x,
           refline_waypoints_.poses[MaxPoseErrId].pose.position.y);
  ROS_INFO("init position: %f, %f", path.poses[MaxPoseErrId].pose.position.x,
           path.poses[MaxPoseErrId].pose.position.y);

  // ROS_INFO("No ERROR:%d", j);
  if (q == refline_waypoints_.poses.size()) return false;
  return true;
}

}  // namespace referenceLine
