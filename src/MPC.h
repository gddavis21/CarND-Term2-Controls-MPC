#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Utility.h"

struct VehicleState
{
    double x;
    double y;
    double psi;
    double v;
};

struct VehicleActuators
{
    double steer;
    double accel;
};

// Use kinematic model to predict future vehicle state, given initial 
// state and actuator values.
VehicleState PredictVehicleState(
    const VehicleState &initial_state,      // initial vehicle state
    const VehicleActuators &actuators,      // current steering & throttle values
    double Lf,                              // vehicle length parameter
    double dt);                             // prediction delay (in seconds)

// Apply Model Predictive Control optimization to compute optimal updates
// to steering angle and throttle, given a reference trajectory, reference
// velocity and initial vehicle state.
bool MPC_UpdateActuators(
    const Polynomial &ref_traj,             // reference trajectory (m)
    double ref_velocity,                    // reference velocity (m/s)
    double Lf,                              // vehicle length parameter
    const VehicleState &initial_state,      // initial vehicle state
    VehicleActuators &pred_actuators,       // output steering & throttle
    std::vector<double> &pred_traj_x,       // output predicted x position
    std::vector<double> &pred_traj_y);      // output predicted y position

#endif /* MPC_H */
