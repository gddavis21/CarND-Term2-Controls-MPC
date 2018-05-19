#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Utility.h"

struct VehicleState
{
    double x;
    double y;
    double psi;
    double v;

    double CrossTrackError(const Polynomial &traj) const {
        return traj.Evaluate(x) - y;
    }

    double OrientationError(const Polynomial &traj) const {
        return psi - atan(traj.Derivative(x));
    }
};

struct VehicleActuators
{
    double steer;
    double accel;
};

// Solve the model given an initial state and polynomial trajectory.
// Return the first actuations.
VehicleActuators SolveMPC(const VehicleState &state, const Polynomial &traj);

#endif /* MPC_H */
