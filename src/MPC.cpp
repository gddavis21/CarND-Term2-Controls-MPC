#include "MPC.h"
#include <vector>
#include <cfloat>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;
using CppAD::AD;

const size_t N = 20;            // Number of time-steps to predict
// const double H = 3.0;           // prediction horizon (seconds)
// const double PRED_STEP = H/N;   // prediction time-step (seconds)

const double H = 80.0;  // horizon in meters

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

class CostFunc 
{
private:
    double _Lf;
    const Polynomial &_ref_traj;  // reference trajectory
    double _ref_velocity, _cur_velocity;
    double _weight_steering_angle, _weight_rate_change_steering;

public:
    CostFunc(
        double Lf,  // vehicle length parameter
        const Polynomial &ref_traj,
        double ref_velocity,
        double cur_velocity) : _ref_traj(ref_traj)
    {
        _Lf = Lf;
        _ref_velocity = ref_velocity;
        _cur_velocity = std::max(cur_velocity, mph_to_mps(10));

        // The cost function weights for steering angle and rate of steering
        // change are velocity dependent.
        _weight_steering_angle = Weight_SteeringAngle(_cur_velocity);
        _weight_rate_change_steering = Weight_RateChangeSteering(_cur_velocity);
    }

    // compute velocity-adaptive weight parameter for steering angle cost term
    static double Weight_SteeringAngle(double velocity)
    {
        // Use several manually tuned velocity vs. weight samples and linear
        // interpolation to compute a specific weight for the given velocity.
        const double V[] = {
            mph_to_mps(10),
            mph_to_mps(40),
            mph_to_mps(55),
            mph_to_mps(70),
            mph_to_mps(85),
            mph_to_mps(100)
        };
        const double W[] = { 1000, 4000, 40000, 100000, 210000, 360000 };
        static LinearInterpolator1D weight(6, &V[0], &W[0]);
        return weight.Interpolate(velocity);
    }

    // compute velocity-adaptive weight parameter for rate of steering change
    static double Weight_RateChangeSteering(double velocity)
    {
        // Use several manually tuned velocity vs. weight samples and linear
        // interpolation to compute a specific weight for the given velocity.
        const double V[] = {
            mph_to_mps(10),
            mph_to_mps(40),
            mph_to_mps(55),
            mph_to_mps(70),
            mph_to_mps(85),
            mph_to_mps(100)
        };
        const double W[] = { 1000, 4000, 40000, 100000, 210000, 360000 };
        static LinearInterpolator1D weight(6, &V[0], &W[0]);
        return weight.Interpolate(velocity);
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector& fg, const ADvector& vars) 
    {
        // TODO: implement MPC
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.

        // cost is stored in fg[0]
        fg[0] = 0;

        // add reference-state cost
        for (size_t t=0; t < N; t++) {
            // It's essential to optimize over cross-track and orientation error
            // out to the horizon, but helps to weight the nearer terms more heavily
            // than the further terms. We'll gradually scale these weights from 
            // (1+k) to (1-k) for 0<k<1.
            double k = 0.3;
            double w = 1.0 + k*(1.0 - 2*double(t)/(N-1));
            fg[0] += w*CppAD::pow(vars[cte_start + t], 2);
            fg[0] += w*CppAD::pow(vars[epsi_start + t], 2);

            // Also penalize for velocity error, but use much smaller weight.
            // Otherwise the car won't slow down enough around sharp turns.
            fg[0] += 0.1*CppAD::pow(vars[v_start + t] - _ref_velocity, 2);
        }

        // penalize actuator changes
        for (size_t t=0; t < N-1; t++) {
            fg[0] += _weight_steering_angle*CppAD::pow(vars[delta_start + t], 2);
            fg[0] += 0.1*CppAD::pow(vars[a_start + t], 2);
        }

        // penalize rates of actuator changes
        for (size_t t=0; t < N-2; t++) {
            fg[0] += _weight_rate_change_steering*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += 0.2*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }

        //
        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // Compute time-step duration (dt) from horizon distance, current velocity
        // and number of time steps.
        double T = H/_cur_velocity;  // time to horizon
        double dt = T/N;
        
        VectorXd C = _ref_traj.GetCoefficients();

        // The rest of the constraints
        for (size_t t = 1; t < N; t++) 
        {
            AD<double> x0 = vars[x_start + t - 1];
            AD<double> x1 = vars[x_start + t];

            AD<double> y0 = vars[y_start + t - 1];
            AD<double> y1 = vars[y_start + t];

            AD<double> psi0 = vars[psi_start + t - 1];
            AD<double> psi1 = vars[psi_start + t];

            AD<double> v0 = vars[v_start + t - 1];
            AD<double> v1 = vars[v_start + t];

            AD<double> cte0 = vars[cte_start + t - 1];
            AD<double> cte1 = vars[cte_start + t];

            AD<double> epsi0 = vars[epsi_start + t - 1];
            AD<double> epsi1 = vars[epsi_start + t];

            AD<double> del0 = vars[delta_start + t - 1];
            AD<double> a0 = vars[a_start + t - 1];

            AD<double> f0 = C[0] + C[1]*x0 + C[2]*x0*x0 + C[3]*x0*x0*x0;
            AD<double> psides0 = CppAD::atan(C[1] + 2*C[2]*x0 + 3*C[3]*x0*x0);

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass
            // these to the solver.

            // Setup the rest of the model constraints
            fg[1 + x_start + t] = x1 - (x0 + v0*CppAD::cos(psi0)*dt);
            fg[1 + y_start + t] = y1 - (y0 + v0*CppAD::sin(psi0)*dt);
            fg[1 + psi_start + t] = psi1 - (psi0 + v0*del0*dt/_Lf);
            fg[1 + v_start + t] = v1 - (v0 + a0*dt);
            fg[1 + cte_start + t] = cte1 - ((y0-f0) + v0*CppAD::sin(epsi0)*dt);
            fg[1 + epsi_start + t] = epsi1 - ((psi0-psides0) + v0*del0*dt/_Lf);
        }
    }
};

namespace
{
    // options for IPOPT solver
    std::string SolverOptions()
    {
        std::string options;
        // Uncomment this if you'd like more print information
        options += "Integer print_level  0\n";
        // NOTE: Setting sparse to true allows the solver to take advantage
        // of sparse routines, this makes the computation MUCH FASTER. If you
        // can uncomment 1 of these and see if it makes a difference or not but
        // if you uncomment both the computation time should go up in orders of
        // magnitude.
        options += "Sparse  true        forward\n";
        options += "Sparse  true        reverse\n";
        // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
        // Change this as you see fit.
        options += "Numeric max_cpu_time          0.5\n";

        return options;
    }
}

VehicleState PredictVehicleState(
    const VehicleState &initial_state,
    const VehicleActuators &actuators,
    double Lf,
    double dt)
{
    VehicleState pred_state = initial_state;

    if (dt > 0.0)
    {
        double psi = initial_state.psi;
        double v = initial_state.v;
        double s = actuators.steer;
        double a = actuators.accel;
        
        pred_state.x += v*cos(psi)*dt;
        pred_state.y += v*sin(psi)*dt;
        pred_state.psi += v*s*dt/Lf;
        pred_state.v += a*dt;
    }

    return pred_state;
}

bool MPC_UpdateActuators(
    const Polynomial &ref_traj,
    double ref_velocity,
    double Lf,  // vehicle length parameter
    const VehicleState &initial_state, 
    VehicleActuators &pred_actuators,
    vector<double> &pred_traj_x,
    vector<double> &pred_traj_y)
{
    // Extract initial state.
    double x = initial_state.x;
    double y = initial_state.y;
    double psi = initial_state.psi;
    double v = initial_state.v;

    // We're going to add cross-track error and orientation error to the
    // optimized state variables, so we need to compute initial values.
    double ref_y = ref_traj.Evaluate(x);
    double ref_psi = atan(ref_traj.Derivative(x));
    double cte = y - ref_y;
    double epsi = psi - ref_psi;

    // number of independent variables & constraints
    size_t n_vars = 6*N + 2*(N-1);
    size_t n_constraints = 6*N;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    typedef CPPAD_TESTVECTOR(double) Dvector;
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    // set initial state variables
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    // variable lower/upper bounds
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // make non-actuator variables (effectively) unbounded
    for (size_t i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -DBL_MAX/2;
        vars_upperbound[i] = DBL_MAX/2;
        // vars_lowerbound[i] = -1.0e19;
        // vars_upperbound[i] = 1.0e19;
    }

    // Constrain steering angle to [-25deg, 25deg]
    double STEERING_LIMIT = deg_to_rad(25.0);

    for (size_t i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -STEERING_LIMIT;
        vars_upperbound[i] = STEERING_LIMIT;
    }

    // Constraint throttle to [-1,1]
    for (size_t i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (size_t i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    CostFunc costFunc(Lf, ref_traj, ref_velocity, v);

    // solve the problem
    CppAD::ipopt::solve_result<Dvector> solution;

    CppAD::ipopt::solve<Dvector, CostFunc>(
        SolverOptions(), 
        vars, 
        vars_lowerbound, 
        vars_upperbound, 
        constraints_lowerbound,
        constraints_upperbound, 
        costFunc, 
        solution);  // OUTPUT

    // diagnostic output
    if (solution.status != CppAD::ipopt::solve_result<Dvector>::success)
    {
        std::cout << "Solver failed" << std::endl;
        return false;
    } 

    std::cout << "Cost: " << solution.obj_value << std::endl;

    // Return the first optimized actuator values.
    pred_actuators.steer = solution.x[delta_start];
    pred_actuators.accel = solution.x[a_start];

    // Also return predicted x/y positions.
    for (size_t t=1; t < N; t++)
    {
        pred_traj_x.push_back(solution.x[x_start + t]);
        pred_traj_y.push_back(solution.x[y_start + t]);
    }

    return true;
}
