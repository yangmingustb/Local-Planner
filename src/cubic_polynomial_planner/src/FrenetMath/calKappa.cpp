//
// Created by ustb on 19-7-9.
//

#include "calKappa.h"
#include <array>
#include <cmath>

namespace lattice_planner {

double trajectory_kappa(const Node node, const Node next_node,
                        std::vector<CubicCoefficients>& coefficients) {
  frenetToCartesian frtToCrt;
  frtToCrt.setParameters(coefficients);

  FrenetPose start;
  start.s = node.s_;
  start.rho = node.rho_;
  start.heading = 0.0 * M_PI / 180.0;

  FrenetPose end;
  end.s = next_node.s_;
  end.rho = next_node.rho_;
  end.heading = 0.0 * M_PI / 180.0;

  CubicPolynomial cubic(start, end);
  auto frenet_path = cubic.FrenetCubicPolynomial().frtPath_;
  // ROS_INFO("cubic.computeFrenet() is completed...");
  // cout<<"-------------------s,rho,theta-size()-------------"<<endl;
  // cout<<s.size()<<"-"<<rho.size()<<"-"<<theta.size()<<endl;

  std::vector<double> kappa_set;
  for (int i = 0; i < (frenet_path.size() - 2); i = i + 5) {
    double x0, x1, x2, y0, y1, y2, k1, k2, k3, s0, rho0, s1, rho1, s2, rho2,
        theta_rho0, theta_rho1, theta_rho2;
    geometry_msgs::PoseStamped cartesian_pose;
    FrenetPose p0, p1, p2;
    p0.s = frenet_path[i].s;
    p0.rho = frenet_path[i].rho;
    p0.heading = frenet_path[i].heading;
    p1.s = frenet_path[i + 1].s;
    p1.rho = frenet_path[i + 1].rho;
    p1.heading = frenet_path[i + 1].heading;
    p2.s = frenet_path[i + 2].s;
    p2.rho = frenet_path[i + 2].rho;
    p2.heading = frenet_path[i + 2].heading;

    cartesian_pose = frtToCrt.transform(p0);
    // ROS_INFO("frenet_to_cartesian() is completed...");
    x0 = cartesian_pose.pose.position.x;
    y0 = cartesian_pose.pose.position.y;
    cartesian_pose = frtToCrt.transform(p1);
    x1 = cartesian_pose.pose.position.x;
    y1 = cartesian_pose.pose.position.y;
    cartesian_pose = frtToCrt.transform(p2);
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

  //    cout<<"--------------------kappa_size-----------"<<endl;
  //    cout<<kappa_set.size()<<endl;

  double sum_kappa = 0.0;
  for (auto i : kappa_set) {
    sum_kappa = sum_kappa + pow(i, 2);
  }
  double mean_kappa = sum_kappa / kappa_set.size();

  //    cout<<"---------------mean kappa-----------"<<endl;
  //    cout<<mean_kappa<<endl;

  return mean_kappa;
}
}