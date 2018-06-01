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

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// target vehicle speed
const double REF_VELOCITY = mph_to_mps(85.0);  // 80 MPH

// simulated actuator latency (must be 100ms for project submission)
const unsigned int LATENCY_ms = 100;

// Turn on to visualize reference trajectory and predicted vehicle position
// in the simulator.
const bool VISUALIZE = true;

int main() 
{
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
            // cout << "unrecognized message" << endl;
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
            // cout << "ignoring simulator message: " << event << endl;
            return;
        } 
        
        // extract input data from simulator message
        // j[1] is the data JSON object
        vector<double> waypts_x = j[1]["ptsx"];     // waypoints x values
        vector<double> waypts_y = j[1]["ptsy"];     // waypoints y values
        double px = j[1]["x"];                  // vehicle current x position
        double py = j[1]["y"];                  // vehicle current y position
        double psi = j[1]["psi"];               // vehicle current orientation
        double speed = j[1]["speed"];               // vehicle current velocity
        double steering_angle = j[1]["steering_angle"];
        double throttle = j[1]["throttle"];

        // initialize state/actuator data structures
        //  - convert speed from MPH to m/s
        //  - change steering angle from CW to CCW
        VehicleState veh_state = { px, py, psi, mph_to_mps(speed) };
        VehicleActuators veh_actuators = { -steering_angle, throttle };

        size_t n_waypts = waypts_x.size();

        // transform waypoints from map to vehicle coordinates
        CoordFrame2D veh_frame(veh_state.x, veh_state.y, veh_state.psi);
        veh_frame.GlobalToLocal(n_waypts, &waypts_x[0], &waypts_y[0]);

        // transform vehicle state to vehicle coordinates
        veh_state.x = 0;
        veh_state.y = 0;
        veh_state.psi = 0;

        // best-fit cubic polynomial trajectory to waypoints
        Polynomial ref_traj(3, n_waypts, &waypts_x[0], &waypts_y[0]);

        // output data we'll send back to simulator
        vector<double> ref_traj_x, ref_traj_y;  // reference trajectory
        vector<double> mpc_traj_x, mpc_traj_y;  // predicted trajectory
        double steer_value = 0;
        double throttle_value = 0;

        // populate reference trajectory visualization data (want to view in
        // simulator even if MPC optimizer fails)
        if (VISUALIZE)
        {
            for (double x = 5.0; x < 50.0; x += 2.0)
            {
                double y = ref_traj.Evaluate(x);
                ref_traj_x.push_back(x);
                ref_traj_y.push_back(y);
            }
        }

        // Model actuator latency by optimizing actuators against model-predicted 
        // state (predicted at time t = now + latency).
        double latency_sec = LATENCY_ms / 1000.0;
        veh_state = PredictVehicleState(veh_state, veh_actuators, Lf, latency_sec);

        // Apply MPC optimization to compute new steering angle and throttle (and 
        // predicted vehicle position for visualization).
        VehicleActuators pred_actuators;
        vector<double> pred_traj_x, pred_traj_y;

        if (MPC_UpdateActuators(
            ref_traj, 
            REF_VELOCITY, 
            Lf,
            veh_state, 
            pred_actuators,     // output
            pred_traj_x,        // output
            pred_traj_y))       // output
        {
            steer_value = -pred_actuators.steer;  // change steering angle CCW to CW
            throttle_value = pred_actuators.accel;

            if (VISUALIZE)
            {
                mpc_traj_x = pred_traj_x;
                mpc_traj_y = pred_traj_y;
            }
        }

        json msgJson;
        // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
        // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
        msgJson["steering_angle"] = steer_value / deg_to_rad(25);
        msgJson["throttle"] = clamp(throttle_value, -1.0, 1.0);

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
