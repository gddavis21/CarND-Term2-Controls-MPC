#include "Utility.h"

using namespace Eigen;
using CppAD::AD;

CoordFrame2D::CoordFrame2D(double org_x, double org_y, double orient) 
{
    _orgX = org_x;
    _orgY = org_y;
    _cosR = cos(orient);
    _sinR = sin(orient);
}

void CoordFrame2D::GlobalToLocal(double &x, double &y) const
{
    double a = x - _orgX;
    double b = y - _orgY;
    x = a*_cosR + b*_sinR;
    y = b*_cosR - a*_sinR; 
}

void CoordFrame2D::LocalToGlobal(double &x, double &y) const
{
    double a = x;
    double b = y;
    x = a*_cosR - b*_sinR + _orgX;
    y = a*_sinR + b*_cosR + _orgY;
}

void CoordFrame2D::GlobalToLocal(size_t count, double *x, double *y) const
{
    for (size_t i=0; i < count; i++) {
        GlobalToLocal(x[i], y[i]);
    }
}

void CoordFrame2D::LocalToGlobal(size_t count, double *x, double *y) const
{
    for (size_t i=0; i < count; i++) {
        LocalToGlobal(x[i], y[i]);
    }
}

namespace 
{
    // Fit a polynomial.
    // Adapted from
    // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
    VectorXd polyfit(
        const VectorXd &xvals, 
        const VectorXd &yvals,
        size_t degree)
    {
        assert(xvals.size() == yvals.size());
        assert(degree >= 1 && degree <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), degree + 1);

        for (int i = 0; i < xvals.size(); i++) {
            A(i, 0) = 1.0;
        }

        for (int j = 0; j < xvals.size(); j++) {
            for (int i = 0; i < degree; i++) {
                A(j, i + 1) = A(j, i) * xvals(j);
            }
        }

        return A.householderQr().solve(yvals);
    }
}

Polynomial::Polynomial(
    size_t degree,
    size_t count,
    const double *xvals, 
    const double *yvals)
{
    Map<const VectorXd> mx(xvals, count);
    Map<const VectorXd> my(yvals, count);
    _coeffs = polyfit(mx, my, degree);
}

// evaluate polynomial
double Polynomial::Evaluate(double x) const
{
    size_t degree = _coeffs.size() - 1;
    double result = 0.0;

    for (size_t i = 0; i <= degree; i++) {
        result += _coeffs[i] * pow(x, i);
    }

    return result;
}

// evaluate polynomial 1st derivative
double Polynomial::Derivative(double x) const
{
    size_t degree = _coeffs.size() - 1;
    double result = 0.0;

    for (size_t i = 1; i <= degree; i++) {
        result += i * _coeffs[i] * pow(x, i-1);
    }

    return result;
}

// Evaluate polynomial CppAD expression
AD<double> Polynomial::Evaluate(const AD<double> &x) const
{
    size_t degree = _coeffs.size() - 1;
    AD<double> result = 0.0;

    for (size_t i = 0; i <= degree; i++) {
        result += _coeffs[i] * CppAD::pow(x, i);
    }

    return result;
}

// Evaluate polynomial 1st derivative CppAD expression
AD<double> Polynomial::Derivative(const AD<double> &x) const
{
    size_t degree = _coeffs.size() - 1;
    AD<double> result = 0.0;

    for (size_t i = 1; i <= degree; i++) {
        result += i * _coeffs[i] * CppAD::pow(x, i-1);
    }

    return result;
}
