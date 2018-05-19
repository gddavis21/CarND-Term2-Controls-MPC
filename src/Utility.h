#ifndef UTILITY_H
#define UTILITY_H

#include <math.h>
#include "Eigen-3.3/Eigen/Dense"
#include <cppad/cppad.hpp>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

class CoordFrame2D
{
public:
    CoordFrame2D(double org_x, double org_y, double orient);

    void GlobalToLocal(double &x, double &y) const;
    void LocalToGlobal(double &x, double &y) const;

    void GlobalToLocal(size_t count, double *x, double *y) const;
    void LocalToGlobal(size_t count, double *x, double *y) const;

private:
    double _orgX, _orgY, _cosR, _sinR;
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

    CppAD::AD<double> Evaluate(const CppAD::AD<double> &x) const;
    CppAD::AD<double> Derivative(const CppAD::AD<double> &x) const;

private:
    Eigen::VectorXd _coeffs;
};

#endif /* UTILITY_H */