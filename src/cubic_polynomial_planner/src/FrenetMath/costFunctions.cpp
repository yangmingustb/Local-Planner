//
// Created by ustb on 19-7-8.
//

#include "costFunctions.h"
#include <cmath>
// #include <yaml-cpp/yaml.h>

namespace lattice_planner {

costFunction::costFunction(std::vector<CubicCoefficients> &coefficients) {
  coefficients_ = coefficients;
}

void costFunction::setParameters(std::vector<CubicCoefficients> &coefficients,
                                 double refline,
                                 std::vector<FrenetPose> obstacle,
                                 frenetToCartesian &frtToCrt) {
  coefficients_ = coefficients;
  reflineRhoValue_ = refline;
  obstacles_ = obstacle;
  frtToCrt_ = frtToCrt;
}

double costFunction::total_cost(Node start, Node end) {
  start_ = start;
  end_ = end;
  frtStart_.s = start_.s_;
  frtStart_.rho = start_.rho_;
  frtStart_.heading = 0.0 * M_PI / 180.0;

  frtEnd_.s = end_.s_;
  frtEnd_.rho = end_.rho_;
  frtEnd_.heading = 0.0 * M_PI / 180.0;

  CubicPolynomial cubic(frtStart_, frtEnd_);
  frtPath_ = cubic.FrenetCubicPolynomial();
  double cost1 = kappa_cost();
  double cost2 = reference_line_cost();
  double cost3 = collision_risk();

  // ROS_INFO("costFunction method is complted...");
  // cout<<"cost1:"<< cost1<<"cost2:"<<cost2<<"cost3:"<<cost3<<endl;

  // double alpha1,alpha2,alpha3;
  double alpha1 = 100;
  double alpha2 = 1;
  double alpha3 = 10;
  double alpha4 = 0.0;
  // ros::param::get("/lattice_planner/alpha1", alpha1);
  // ros::param::get("/lattice_planner/alpha2", alpha2);
  // ros::param::get("/lattice_planner/alpha3", alpha3);
  double cost = alpha1 * cost1 + alpha2 * cost2 + alpha3 * cost3;
  return cost;
}

double costFunction::kappa_cost() {
  double mean_kappa = trajectory_kappa();
  // ROS_INFO("trajectory_kappa() is completed...");
  return mean_kappa;
}

double costFunction::reference_line_cost() {
  double dis1 = fabs(start_.rho_ - reflineRhoValue_);
  double dis2 = fabs(end_.rho_ - reflineRhoValue_);

  double cost = (dis1 + dis2) / 2.0;
  return cost;
}

double costFunction::collision_risk() {
  double r_circle = 1.0;
  double obstacle_inflation = 1.5;
  // ros::param::get("/lattice_planner/r_circle", r_circle);
  // std::cout<<"r_circle:"<<r_circle<<std::endl;
  // ros::param::get("/lattice_planner/obstacle_inflation", obstacle_inflation);
  double dis = 100.0;
  for (size_t i = 0; i < (frtPath_.frtPath_.size() - 1); i = i + 5) {
    for (size_t j = 0; j != obstacles_.size(); j++) {
      double TmpDis =
          sqrt(pow((frtPath_.frtPath_[i].s - obstacles_[j].s), 2) +
               pow((frtPath_.frtPath_[i].rho - obstacles_[j].rho), 2));
      if (TmpDis < (r_circle + obstacle_inflation)) {
        if (TmpDis < dis) dis = TmpDis;
      }
    }
  }
  double cost = 1 / (dis + 0.001);
  return cost;
}

double costFunction::trajectory_kappa() {
  std::vector<double> kappa_set;
  for (int i = 0; i < (frtPath_.frtPath_.size() - 2); i = i + 5) {
    double x0, x1, x2, y0, y1, y2, k1, k2, k3, s0, rho0, s1, rho1, s2, rho2,
        theta_rho0, theta_rho1, theta_rho2;
    geometry_msgs::PoseStamped cartesian_pose;
    FrenetPose p0, p1, p2;
    p0.s = frtPath_.frtPath_[i].s;
    p0.rho = frtPath_.frtPath_[i].rho;
    p0.heading = frtPath_.frtPath_[i].heading;
    p1.s = frtPath_.frtPath_[i + 1].s;
    p1.rho = frtPath_.frtPath_[i + 1].rho;
    p1.heading = frtPath_.frtPath_[i + 1].heading;
    p2.s = frtPath_.frtPath_[i + 2].s;
    p2.rho = frtPath_.frtPath_[i + 2].rho;
    p2.heading = frtPath_.frtPath_[i + 2].heading;

    cartesian_pose = frtToCrt_.transform(p0);
    // ROS_INFO("frenet_to_cartesian() is completed...");
    x0 = cartesian_pose.pose.position.x;
    y0 = cartesian_pose.pose.position.y;
    cartesian_pose = frtToCrt_.transform(p1);
    x1 = cartesian_pose.pose.position.x;
    y1 = cartesian_pose.pose.position.y;
    cartesian_pose = frtToCrt_.transform(p2);
    x2 = cartesian_pose.pose.position.x;
    y2 = cartesian_pose.pose.position.y;

    k1 = (x1 - x0) * (y2 - 2 * y1 + y0);
    k2 = (y1 - y0) * (x2 - 2 * x1 + x0);
    k3 = pow((pow((x1 - x0), 2) + pow((y1 - y0), 2)), 1.5);

    if (k3 == 0.0)
      kappa_set.push_back(0.0);

    else
      kappa_set.push_back((k1 - k2) / k3);
  }

  // cout<<"--------------------kappa_size-----------"<<endl;
  // cout<<kappa_set.size()<<endl;

  double sum_kappa = 0.0;
  for (auto i : kappa_set) {
    sum_kappa = sum_kappa + pow(i, 2);
  }
  double mean_kappa = sum_kappa / kappa_set.size();

  // cout<<"---------------mean kappa-----------"<<endl;
  // cout<<mean_kappa<<endl;

  return mean_kappa;
}

}  // namespace lattice_planner
