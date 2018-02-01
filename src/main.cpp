#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "Tools.h"
#include "json.hpp"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

// for convenience
using json = nlohmann::json;

int main() {
  const int latency_ms = 100;
  const int polyOrder = 3;
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    bool debug = false;
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
