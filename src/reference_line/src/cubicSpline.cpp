//
// Created by ustb on 19-7-15.
//

#include "cubicSpline.h"

namespace cubicSpline {

std::vector<double> vec_diff(std::vector<double> input) {
  std::vector<double> output;
  for (unsigned int i = 1; i < input.size(); i++) {
    output.push_back(input[i] - input[i - 1]);
  }
  return output;
}

std::vector<double> calculate_sum(const std::vector<double> &input) {
  std::vector<double> output;
  float temp = 0;
  for (auto i : input) {
    temp += i;
    output.push_back(temp);
  }
  return output;
}

Spline::Spline(std::vector<double> x, std::vector<double> y)
    : x(x), y(y), nx(x.size()), h(vec_diff(x)), a(y) {
  Eigen::MatrixXd A = calc_A();
  Eigen::VectorXd B = calc_B();
  Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(B);
  double *c_pointer = c_eigen.data();
  // Eigen::Map<Eigen::VectorXf>(c, c_eigen.rows(), 1) = c_eigen;
  c.assign(c_pointer, c_pointer + c_eigen.rows());

  for (int i = 0; i < nx - 1; i++) {
    d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
    b.push_back((a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3.0);
  }
}

double Spline::calc(double t) {
  if (t < x.front() || t > x.back()) {
    throw std::invalid_argument("received value out of the pre-defined range");
  }
  int seg_id = binarySearch(t, 0, nx);
  double dx = t - x[seg_id];
  return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx +
         d[seg_id] * dx * dx * dx;
}

double Spline::calc_d(double t) {
  if (t < x.front() || t > x.back()) {
    throw std::invalid_argument("received value out of the pre-defined range");
  }
  int seg_id = binarySearch(t, 0, nx - 1);
  double dx = t - x[seg_id];
  return b[seg_id] + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
}

double Spline::calc_dd(double t) {
  if (t < x.front() || t > x.back()) {
    throw std::invalid_argument("received value out of the pre-defined range");
  }
  int seg_id = binarySearch(t, 0, nx);
  double dx = t - x[seg_id];
  return 2 * c[seg_id] + 6 * d[seg_id] * dx;
}

Eigen::MatrixXd Spline::calc_A() {
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
  A(0, 0) = 1;
  for (int i = 0; i < nx - 1; i++) {
    if (i != nx - 2) {
      A(i + 1, i + 1) = 2 * (h[i] + h[i + 1]);
    }
    A(i + 1, i) = h[i];
    A(i, i + 1) = h[i];
  }
  A(0, 1) = 0.0;
  A(nx - 1, nx - 2) = 0.0;
  A(nx - 1, nx - 1) = 1.0;
  return A;
}

Eigen::VectorXd Spline::calc_B() {
  Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
  for (int i = 0; i < nx - 2; i++) {
    B(i + 1) =
        3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
  }
  return B;
}

int Spline::binarySearch(double t, int start, int end) {
  int mid = (start + end) / 2;
  if (t == x[mid] || end - start <= 1) {
    return mid;
  } else if (t > x[mid]) {
    return binarySearch(t, mid, end);
  } else {
    return binarySearch(t, start, mid);
  }
}

Spline2D::Spline2D(std::vector<double> x, std::vector<double> y) {
  s = calc_s(x, y);
  sx = Spline(s, x);
  sy = Spline(s, y);
}

Poi_d Spline2D::calculatePosition(double s_t) {
  double x = sx.calc(s_t);
  double y = sy.calc(s_t);
  return {{x, y}};
}

double Spline2D::calculateCurvature(double s_t) {
  double dx = sx.calc_d(s_t);
  double ddx = sx.calc_dd(s_t);
  double dy = sy.calc_d(s_t);
  double ddy = sy.calc_dd(s_t);
  return (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
}

double Spline2D::calculateHeading(double s_t) {
  double dx = sx.calc_d(s_t);
  double dy = sy.calc_d(s_t);
  return std::atan2(dy, dx);
}

std::vector<double> Spline2D::calc_s(std::vector<double> x,
                                     std::vector<double> y) {
  std::vector<double> ds;
  std::vector<double> out_s{0};
  std::vector<double> dx = vec_diff(x);
  std::vector<double> dy = vec_diff(y);

  for (unsigned int i = 0; i < dx.size(); i++) {
    ds.push_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
  }

  std::vector<double> cum_ds = calculate_sum(ds);
  out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
  return out_s;
}
}

#ifdef BUILD_INDIVIDUAL
int main() {
  std::vector<double> x{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
  std::vector<double> y{0.7, -6, 5, 6.5, 0.0, 5.0, -2.0};
  std::vector<double> r_x;
  std::vector<double> r_y;
  std::vector<double> ryaw;
  std::vector<double> rcurvature;
  std::vector<double> rs;

  Spline2D csp_obj(x, y);
  for (float i = 0; i < csp_obj.s.back(); i += 0.1) {
    std::array<float, 2> point_ = csp_obj.calc_postion(i);
    r_x.push_back(point_[0]);
    r_y.push_back(point_[1]);
    ryaw.push_back(csp_obj.calc_yaw(i));
    rcurvature.push_back(csp_obj.calc_curvature(i));
    rs.push_back(i);
  }
}
#endif