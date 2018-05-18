#include "Polynomial.h"

using namespace Eigen;
using CppAD::AD;

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
    const Eigen::VectorXd &xvals, 
    const Eigen::VectorXd &yvals,
    size_t degree)
{
    _coeffs = polyfit(xvals, yvals, degree);
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
