#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen/Dense"
#include "Utility.h"

struct VehicleState
{
    double x;
    double y;
    double psi;
    double v;
};

struct MPC_Results
{
    double steer;
    double accel;
    std::vector<double> traj_x;
    std::vector<double> traj_y;
};

// Solve the model given an initial state and polynomial trajectory.
// Return the first actuations.
bool MPC_Solve(
    const VehicleState &state, 
    //const Polynomial &ref_traj,
    const Eigen::VectorXd &ref_traj,
    MPC_Results &results);

#endif /* MPC_H */
