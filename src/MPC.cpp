#include "MPC.h"
#include <vector>
#include <cfloat>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

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

const double ref_velocity = 30.0;

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

class FG_eval 
{
private:
    // fitted polynomial trajectory
    Polynomial _traj;

public:
    FG_eval(const Polynomial &traj) : _traj(traj) {}

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
            fg[0] += CppAD::pow(vars[cte_start + t], 2);
            fg[0] += CppAD::pow(vars[epsi_start + t], 2);
            fg[0] += CppAD::pow(vars[v_start + t] - ref_velocity, 2);
        }

        // penalize actuator changes
        for (size_t t=0; t < N-1; t++) {
            fg[0] += CppAD::pow(vars[delta_start + t], 2);
            fg[0] += CppAD::pow(vars[a_start + t], 2);
        }

        // penalize rate of actuator changes
        for (size_t t=0; t < N-2; t++) {
            fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
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

        // The rest of the constraints
        for (int t = 1; t < N; t++) 
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

            AD<double> f0 = _traj.Evaluate(x0);
            AD<double> psides0 = CppAD::atan(_traj.Derivative(x0));

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass
            // these to the solver.

            // Setup the rest of the model constraints
            fg[1 + x_start + t] = x1 - (x0 + v0*CppAD::cos(psi0)*dt);
            fg[1 + y_start + t] = y1 - (y0 + v0*CppAD::sin(psi0)*dt);
            fg[1 + psi_start + t] = psi1 - (psi0 + v0*del0*dt/Lf);
            fg[1 + v_start + t] = v1 - (v0 + a0*dt);
            fg[1 + cte_start + t] = cte1 - ((f0-y0) + v0*CppAD::sin(epsi0)*dt);
            fg[1 + epsi_start + t] = epsi1 - ((psi0-psides0) + v0*del0*dt/Lf);
        }
    }
};

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

VehicleActuators SolveMPC(const VehicleState &state, const Polynomial &traj) 
{
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // extract current state
    double x = state.x;
    double y = state.y;
    double psi = state.psi;
    double v = state.v;
    double cte = state.CrossTrackError(traj);
    double epsi = state.OrientationError(traj);

    // number of independent variables & constraints
    size_t n_vars = 6*N + 2*(N-1);
    size_t n_constraints = 6*N;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
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
        vars_lowerbound[i] = -DBL_MAX;
        vars_upperbound[i] = DBL_MAX;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    const double STEERING_LIMIT = deg2rad(25.0);
    for (size_t i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -STEERING_LIMIT;
        vars_upperbound[i] = STEERING_LIMIT;
    }

    // -1 <= acceleration <= 1
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
    FG_eval fg_eval(traj);

    // solve the problem
    CppAD::ipopt::solve_result<Dvector> solution;
    VehicleActuators actuators;

    CppAD::ipopt::solve<Dvector, FG_eval>(
        SolverOptions(), 
        vars, 
        vars_lowerbound, 
        vars_upperbound, 
        constraints_lowerbound,
        constraints_upperbound, 
        fg_eval, 
        solution);  // OUTPUT

    // diagnostic output
    if (solution.status == CppAD::ipopt::solve_result<Dvector>::success) 
    {
        auto cost = solution.obj_value;
        std::cout << "Cost " << cost << std::endl;

        // Return the first actuator values. The variables can be accessed with
        // `solution.x[i]`.
        actuators.steer = solution.x[delta_start];
        actuators.accel = solution.x[a_start];
    }
    else 
    {
        std::cout << "Solver failed" << std::endl;
        actuators.steer = actuators.accel = 0;
    }

    return actuators;
}
