#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include "Eigen-3.3/Eigen/Dense"
#include <cppad/cppad.hpp>

class Polynomial 
{
public:
    Polynomial(
        const Eigen::VectorXd &xvals, 
        const Eigen::VectorXd &yvals,
        size_t degree);

    // evaluate polynomial at x
    double Evaluate(double x) const;

    // evaluate polynomial 1st derivative at x
    double Derivative(double x) const;

    CppAD::AD<double> Evaluate(const CppAD::AD<double> &x) const;
    CppAD::AD<double> Derivative(const CppAD::AD<double> &x) const;

private:
    Eigen::VectorXd _coeffs;
};

#endif /* POLYNOMIAL_H */