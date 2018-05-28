#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen/Dense"
#include "Utility.h"

// struct VehicleState
// {
//     double x;
//     double y;
//     double psi;
//     double v;
// };

// struct MPC_Results
// {
//     double steer;
//     double accel;
//     std::vector<double> traj_x;
//     std::vector<double> traj_y;
// };

// // Solve the model given an initial state and polynomial trajectory.
// // Return the first actuations.
// bool MPC_Solve(
//     const VehicleState &state, 
//     //const Polynomial &ref_traj,
//     const Eigen::VectorXd &ref_traj,
//     MPC_Results &results);

class MPC
{
public:
    MPC(double ref_velocity);

    struct VehicleState
    {
        double x;
        double y;
        double psi;
        double v;
    };

    struct Results 
    {
        double steer;
        double accel;
        std::vector<double> traj_x;
        std::vector<double> traj_y;
    };

    bool Solve(
        const VehicleState &state, 
        //const Polynomial &ref_traj,
        const Eigen::VectorXd &ref_traj,
        Results &results) const;

private:
    double _ref_velocity;
    double _fwd_proj_dist;
    Eigen::VectorXd _weight_coeff_change_steer;
    Eigen::VectorXd _weight_coeff_rate_change_steer;
};

#endif /* MPC_H */
