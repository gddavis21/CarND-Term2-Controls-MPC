#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen/Dense"
#include "Utility.h"
#include "MPC.h"
#include "json.hpp"

// for convenience
using namespace std;
using json = nlohmann::json;

// // For converting back and forth between radians and degrees.
// constexpr double pi() { return M_PI; }
// double deg2rad(double x) { return x * pi() / 180; }
// double rad2deg(double x) { return x * 180 / pi(); }

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

// // Evaluate a polynomial.
// double polyeval(Eigen::VectorXd coeffs, double x) {
//   double result = 0.0;
//   for (int i = 0; i < coeffs.size(); i++) {
//     result += coeffs[i] * pow(x, i);
//   }
//   return result;
// }

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

int main() 
{
    const unsigned int LATENCY_ms = 0;
    const bool VISUALIZE = true;

    uWS::Hub h;

    h.onMessage([](
        uWS::WebSocket<uWS::SERVER> ws, 
        char *data, 
        size_t length,
        uWS::OpCode opCode) 
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;

        if (sdata.size() < 3 || sdata[0] != '4' || sdata[1] != '2') {
            cout << "unrecognized message" << endl;
            return;
        } 
        
        string s = hasData(sdata);

        if (s == "") {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            return;
        }

        json j = json::parse(s);
        string event = j[0].get<string>();

        if (event != "telemetry") {
            cout << "ignoring simulator message: " << event << endl;
            return;
        } 
        
        // extract input data from simulator
        // j[1] is the data JSON object
        vector<double> ptsx = j[1]["ptsx"];     // waypoints x values
        vector<double> ptsy = j[1]["ptsy"];     // waypoints y values
        double px = j[1]["x"];                  // vehicle current x position
        double py = j[1]["y"];                  // vehicle current y position
        double psi = j[1]["psi"];               // vehicle current orientation
        double v = j[1]["speed"];               // vehicle current velocity

        // output data we'll send back to simulator
        double steer_value = 0;
        double throttle_value = 0;
        vector<double> ref_traj_x, ref_traj_y;  // reference trajectory
        vector<double> mpc_traj_x, mpc_traj_y;  // predicted trajectory

        // transform waypoints from map to vehicle coordinates
        CoordFrame2D veh_frame(px, py, psi);
        veh_frame.GlobalToLocal(ptsx.size(), &ptsx[0], &ptsy[0]);

        cout << "Waypoints (veh coords)" << endl;
        cout << "x:";
        for (size_t i=0; i < ptsx.size(); i++)
            cout << " " << ptsx[i];
        cout << endl;
        cout << "y:";
        for (size_t i=0; i < ptsy.size(); i++)
            cout << " " << ptsy[i];
        cout << endl;
        
        // best-fit cubic polynomial trajectory to waypoints
        // Polynomial ref_traj(3, ptsx.size(), &ptsx[0], &ptsy[0]);
        Eigen::Map<Eigen::VectorXd> mx(&ptsx[0], ptsx.size());
        Eigen::Map<Eigen::VectorXd> my(&ptsy[0], ptsx.size());
        Eigen::VectorXd ref_traj = polyfit(mx, my, 3);

        if (VISUALIZE)
        {
            for (double x = 5.0; x < 50.0; x += 2.0)
            {
                double y = ref_traj[0] + ref_traj[1]*x + ref_traj[2]*x*x + ref_traj[3]*x*x*x;
                ref_traj_x.push_back(x);
                ref_traj_y.push_back(y);
            }
        }

        // working in vehicle coordinates
        VehicleState veh_state;
        veh_state.x = 0;
        veh_state.y = 0;
        veh_state.psi = 0;
        veh_state.v = mph_to_mps(v);

        MPC_Results mpc_results;

        if (MPC_Solve(veh_state, ref_traj, mpc_results))
        {
            steer_value = -mpc_results.steer;
            throttle_value = mpc_results.accel;

            if (VISUALIZE)
            {
                mpc_traj_x = mpc_results.traj_x;
                mpc_traj_y = mpc_results.traj_y;
            }
        }

        // Calculate steering angle and throttle using MPC.
        // Both are in between [-1, 1].

        json msgJson;
        // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
        // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
        msgJson["steering_angle"] = steer_value / deg_to_rad(25);
        msgJson["throttle"] = throttle_value;

        //Display the MPC predicted trajectory 

        //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
        // the points in the simulator are connected by a Green line

        msgJson["mpc_x"] = mpc_traj_x;
        msgJson["mpc_y"] = mpc_traj_y;

        //Display the waypoints/reference line

        //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
        // the points in the simulator are connected by a Yellow line

        msgJson["next_x"] = ref_traj_x;
        msgJson["next_y"] = ref_traj_y;


        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
        std::cout << msg << std::endl;

        // Latency
        // The purpose is to mimic real driving conditions where
        // the car does not actuate the commands instantly.
        //
        // Feel free to play around with this value but should be to drive
        // around the track with 100ms latency.
        //
        // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
        // SUBMITTING.
        this_thread::sleep_for(chrono::milliseconds(LATENCY_ms));
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](
        uWS::HttpResponse *res, 
        uWS::HttpRequest req, 
        char *data,
        size_t, size_t) 
    {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } 
        else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](
        uWS::WebSocket<uWS::SERVER> ws, 
        int code,
        char *message, size_t length) 
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } 
    else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
