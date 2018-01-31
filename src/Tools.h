#include <vector>
#include <string>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"

std::string hasData(std::string s);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
double last_steer(double val = 0.);
void update_err(double cte, double epsi, double steer, double dsteer);
void applyLatency(double& px, double& py, double& psi, double v, double latency_ms, double steer);
void toCarCoord(std::vector<double>& ptsx, std::vector<double>& ptsy, double& px, double& py, double& psi);
double polyeval(Eigen::VectorXd coeffs, double x);
double polyderiv(Eigen::VectorXd coeffs, double x);
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);
CppAD::AD<double> polyderiv(Eigen::VectorXd coeffs, CppAD::AD<double> x);
