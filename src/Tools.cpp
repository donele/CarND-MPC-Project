#include <iostream>
#include "Tools.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;
using CppAD::AD;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

double last_steer(double val) {
  static double last_steer_ = 0.;
  if(val !=  0.)
    last_steer_ = val;
  return last_steer_;
}

void applyLatency(double& px, double& py, double& psi, double v, double latency_ms, double steer) {
  const double Lf = 2.67;
  double dt = latency_ms / 1000.;
  px += v * cos(psi) * dt;
  py += v * sin(psi) * dt;
  psi -= v / Lf * steer * dt;
}

void toCarCoord(vector<double>& ptsx, vector<double>& ptsy, double& px, double& py, double& psi) {
  vector<double> carx;
  vector<double> cary;

  int n = ptsx.size();
  for(int i = 0; i < n; ++i) {
    double x = cos(-psi) * (ptsx[i] - px) - sin(-psi) * (ptsy[i] - py);
    double y = sin(-psi) * (ptsx[i] - px) + cos(-psi) * (ptsy[i] - py);
    carx.push_back(x);
    cary.push_back(y);
  }

  ptsx = carx;
  ptsy = cary;
  px = 0.;
  py = 0.;
  psi = 0.;
}

void update_err(double cte, double epsi, double steer, double dsteer) {
  static double sum_cte_sq = 0.;
  static double sum_epsi_sq = 0.;
  static double sum_steer_sq = 0.;
  static double sum_dsteer_sq = 0.;
  static int n = 0;
  sum_cte_sq += cte * cte;
  sum_epsi_sq += epsi * epsi;
  sum_steer_sq += steer * steer;
  sum_dsteer_sq += dsteer * dsteer;
  ++n;
  const int maxn = 400;
  if(n > maxn) {
    printf("avg cte %.4f avg epsi %.4f avg steer %.4f avg dsteer %.4f (n=%d)\n",
        sqrt(sum_cte_sq/n), sqrt(sum_epsi_sq/n), sqrt(sum_steer_sq/n), sqrt(sum_dsteer_sq/n), maxn);
    cout.flush();
    exit(0);
  }
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Evaluate a derivative.
double polyderiv(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += i * coeffs[i] * pow(x, i - 1);
  }
  return result;
}

// Evaluate a polynomial.
AD<double> polyeval(Eigen::VectorXd coeffs, AD<double> x) {
  AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

// Evaluate a derivative.
AD<double> polyderiv(Eigen::VectorXd coeffs, AD<double> x) {
  AD<double> result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += i * coeffs[i] * CppAD::pow(x, i - 1);
  }
  return result;
}

