#ifndef UTILITY_H
#define UTILITY_H

#include <math.h>
#include "Eigen/Dense"
#include <cppad/cppad.hpp>

// conversions between degrees and radians
double deg_to_rad(double deg);
double rad_to_deg(double rad);

// conversions between miles/hour and meters/sec
double mph_to_mps(double mph);
double mps_to_mph(double mps);

class CoordFrame2D
{
public:
    CoordFrame2D(double x_offs, double y_offs, double orient);

    void GlobalToLocal(double &x, double &y) const;
    void LocalToGlobal(double &x, double &y) const;

    void GlobalToLocal(size_t count, double *x, double *y) const;
    void LocalToGlobal(size_t count, double *x, double *y) const;

private:
    double _xoffs, _yoffs, _cosA, _sinA;
};

class Polynomial 
{
public:
    Polynomial(
        size_t degree,
        size_t count,
        const double *xvals, 
        const double *yvals);

    // evaluate polynomial at x
    double Evaluate(double x) const;

    // evaluate polynomial 1st derivative at x
    double Derivative(double x) const;

    // CppAD::AD<double> Evaluate(const CppAD::AD<double> &x) const;
    // CppAD::AD<double> Derivative(const CppAD::AD<double> &x) const;

private:
    Eigen::VectorXd _coeffs;
};

#endif /* UTILITY_H */