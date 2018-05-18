#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Polynomial.h"

// Solve the model given an initial state and polynomial trajectory.
// Return the first actuations.
std::vector<double> SolveMPC(
    const Eigen::VectorXd &state, 
    const Polynomial &traj);

#endif /* MPC_H */
