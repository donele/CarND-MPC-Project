#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double last_steer(double val = 0.);
void update_err(double cte, double epsi, double steer, double dsteer);
void applyLatency(double& px, double& py, double& psi, double v, double latency_ms, double steer);
void toCarCoord(vector<double>& ptsx, vector<double>& ptsy, double& px, double& py, double& psi);

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
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
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

int main() {
  const int latency_ms = 100;
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    bool debug = true;
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          toCarCoord(ptsx, ptsy, px, py, psi);
          applyLatency(px, py, psi, v, latency_ms, last_steer());

          int vsize = ptsx.size();
          Eigen::VectorXd vptsx(vsize);
          Eigen::VectorXd vptsy(vsize);
          for(int i = 0; i < vsize; ++i) {
            vptsx[i] = ptsx[i];
            vptsy[i] = ptsy[i];
          }

          int polyOrder = 3;
          auto coeffs = polyfit(vptsx, vptsy, polyOrder);
          double epsi = psi - atan(polyderiv(coeffs, px));
          double cte = py - polyeval(coeffs, px);
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value = vars[2][0];
          double throttle_value = vars[3][0];

          if(debug)
            update_err(cte, epsi, steer_value, steer_value - last_steer());

          last_steer(steer_value);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;


          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for(int i = 0; i < vars[0].size(); ++i) {
            mpc_x_vals.push_back(vars[0][i]);
            mpc_y_vals.push_back(vars[1][i]);
          }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for(int i = 1; i < 30; ++i) {
            double x = i * 5;
            next_x_vals.push_back(x);
            next_y_vals.push_back(polyeval(coeffs, x));
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds(latency_ms));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

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
