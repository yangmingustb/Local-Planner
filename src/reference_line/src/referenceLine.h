/**
 * Created by ustb on 19-7-12.
 input:way-points
 output:coefficients of the arc-length parameterized reference line
 x=a[0]+a[1]*s + a[2]*s^2 + a[3]*s^3;
 y=b[0]+b[1]*s + b[2]*s^2 + b[3]*s^3;
 d_x = a_vec[1] + 2 * a_vec[2] * s + 3 * a_vec[3] * s ** 2
 d_y = b_vec[1] + 2 * b_vec[2] * s + 3 * b_vec[3] * s ** 2
 */

//

#ifndef LATTICEPLANNER_REFERENCELINE_H
#define LATTICEPLANNER_REFERENCELINE_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <array>
#include <iostream>
#include <vector>
#include "cubicSpline.h"

namespace reference_line {

// typedef std::vector<std::vector<double >> coefficients_type;

/**
 * self defined struct
 */
struct arc_length_parameter {
  double s;
  double a0, a1, a2, a3;
  double b0, b1, b2, b3;
};

/**
 * reference line class
 */
class RefLine {
 public:
  RefLine();

  /**
   * select appropriate step between way-points in the reference line
   */
  void SparseWayPoints(std::vector<double> r_x, std::vector<double> r_y,
                       std::vector<double> r_heading, std::vector<double> r_s);

  /**
   * reference line is parameterized by the arc length
   */
  void arcLengthRefLine(std::vector<double> pose, std::vector<double> nextPose,
                        double s0, double sf, std::ofstream &writeFile);

  /**
   * obtain the coefficients of the reference line,
   * this is an API in the RefLine class.
   */
  std::vector<arc_length_parameter> ref_coefficients_output() const;

  /**
   * visualization in rviz, display the reference line
   */
  nav_msgs::Path generateRefLine_inRviz();

  /**
   * calculate the cartesian pose of every point in the reference line.
   */
  geometry_msgs::PoseStamped poses_of_reference_line(double s);

  /**
   * binary search about the coefficients lookup table.
   */
  int binary_search(double s);

  /**
   * debug function.
   * read coefficients lookup table from the file which is made by the
   * RefLine::RefLine().
   */
  nav_msgs::Path readCoefficientsFromFile();

  /**
   * 计算在线计算出的参考线系数，和经过存储到csv文件再读取后的参考线系数，之间的系数误差，
   * 以及依据两者两者生成的参考线位姿误差。
   * 这个函数也是用来debug的。
   */
  bool isSameData(std::vector<arc_length_parameter> &coeff,
                  nav_msgs::Path &path);

 private:
  std::vector<geometry_msgs::Point> waypoints_;
  std::vector<arc_length_parameter> coefficients_;
  std::vector<arc_length_parameter> coefficientsTest_;
  nav_msgs::Path refline_waypoints_;
};

}  // namespace referenceLine

#endif  // LATTICEPLANNER_REFERENCELINE_H
