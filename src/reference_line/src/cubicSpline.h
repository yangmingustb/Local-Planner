//
// Created by ustb on 19-7-15.
//

#ifndef LATTICEPLANNER_CUBICSPLINE_H
#define LATTICEPLANNER_CUBICSPLINE_H

#include <Eigen/Eigen>
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

// using std::vector<double >=std::vector<double >;
using Poi_d = std::array<double, 2>;
using Vec_Poi = std::vector<Poi_d>;

namespace cubicSpline {

/**
 * calculate the first order difference
 * @param input
 * @return
 */
std::vector<double> vec_diff(std::vector<double> input);

/**
 * calculate the sum of a vector
 * @param input
 * @return
 */
std::vector<double> calculate_sum(const std::vector<double> &input);

class Spline {
 public:
  std::vector<double> x;
  std::vector<double> y;
  int nx;
  std::vector<double> h;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  // Eigen::VectorXf c;
  std::vector<double> d;

  Spline(){};
  // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
  Spline(std::vector<double> x, std::vector<double> y);

  double calc(double t);

  double calc_d(double t);

  double calc_dd(double t);

 private:
  Eigen::MatrixXd calc_A();

  Eigen::VectorXd calc_B();

  int binarySearch(double t, int start, int end);
};

class Spline2D {
 public:
  Spline sx;
  Spline sy;
  std::vector<double> s;

  Spline2D(std::vector<double> x, std::vector<double> y);
  Poi_d calculatePosition(double s_t);

  double calculateCurvature(double s_t);

  double calculateHeading(double s_t);

 private:
  std::vector<double> calc_s(std::vector<double> x, std::vector<double> y);
};
}

#endif  // LATTICEPLANNER_CUBICSPLINE_H
