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

struct VehicleActuators
{
    double steer;
    double accel;
};

class MPC
{
public:
    MPC(double Lf);

    bool Predict(
        //const Polynomial &ref_traj,
        const Eigen::VectorXd &ref_traj,
        double ref_velocity,
        const VehicleState &current_state, 
        const VehicleActuators &current_actuators,
        double latency,
        VehicleActuators &pred_actuators,
        std::vector<double> &pred_traj_x,
        std::vector<double> &pred_traj_y) const;

private:
    VehicleState AdjustForLatency(
        const VehicleState &current_state,
        const VehicleActuators &current_actuators,
        double latency) const;

    double _Lf;
};

#endif /* MPC_H */
