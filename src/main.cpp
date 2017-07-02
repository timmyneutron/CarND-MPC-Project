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

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
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
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
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

          // number of points in the waypoints vector
          int n_pts = ptsx.size();

          // initialize std vectors and Eigen class vectors for waypoints in
          // the car reference frame
          std::vector<double> x_car_vec;
          std::vector<double> y_car_vec;
          Eigen::VectorXd x_car_eig(n_pts);
          Eigen::VectorXd y_car_eig(n_pts);

          // transform waypoints from global reference frame
          // to car reference frame and store points in vectors
          for (int i = 0; i < n_pts; ++i)
          {
            double x_diff = ptsx[i] - px;
            double y_diff = ptsy[i] - py;

            double x_car = cos(psi) * x_diff + sin(psi) * y_diff;
            double y_car = -sin(psi) * x_diff + cos(psi) * y_diff;

            x_car_eig[i] = x_car;
            y_car_eig[i] = y_car;

            x_car_vec.push_back(x_car);
            y_car_vec.push_back(y_car);
          }

          // fit a 3rd order polynomial to waypoints in car reference frame
          Eigen::VectorXd coeffs = polyfit(x_car_eig, y_car_eig, 3);

          // set cross track error to negative the y-intercept
          double cte = -coeffs[0];

          // likewise since yaw angle in the car reference frame is always 0,
          // set yaw error to -1 * derivative of the polynomial at x = 0
          double epsi = -atan(coeffs[1]);

          // set state vector for car in the car reference frame,
          // so x, y, and psi are set to 0
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          // call the MPC solver function to calculate the best trajectory
          std::vector<double> vars = mpc.Solve(state, coeffs);

          // convert steering angle from radians to [-1, 1] and set value
          // (max steering angle is 25 degrees = .436332)
          double steer_value = -vars[0] / 0.436332;

          // set throttle value
          double throttle_value = vars[1];

          // initialize json object and set steering and throttle values
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // add predicted trajectory points to vector (shows up as green line in sim)
          for (int i = 0; i < mpc.n_timesteps_; ++i)
          {
            mpc_x_vals.push_back(vars[2 + i]);
            mpc_y_vals.push_back(vars[2 + mpc.n_timesteps_ + i]);
          }

          // add predicted trajectory to json object
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // add waypoints to json object
          msgJson["next_x"] = x_car_vec;
          msgJson["next_y"] = y_car_vec;

          // dump json object in string to send to sim
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          // sleep for 100 milliseconds to simulate latency
          this_thread::sleep_for(chrono::milliseconds(100));

          // send command to sim
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
