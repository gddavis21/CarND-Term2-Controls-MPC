#include "Utility.h"

using namespace Eigen;
using CppAD::AD;

double deg_to_rad(double deg) { 
    return deg * M_PI / 180; 
}

double rad_to_deg(double rad) { 
    return rad * 180 / M_PI; 
}

const static double mm_per_inch = 25.4;
const static double sec_per_hour = 60 * 60;
const static double inch_per_mile = 12 * 5280;

double mph_to_mps(double mph) {
    double inch_per_sec = mph * inch_per_mile / sec_per_hour;
    return inch_per_sec * mm_per_inch / 1000;
}

double mps_to_mph(double mps) {
    // double inch_per_sec = mps * 1000 / mm_per_inch;
    // return inch_per_sec * sec_per_hour / inch_per_mile;
    return mps / mph_to_mps(1.0);
}

CoordFrame2D::CoordFrame2D(double x_org, double y_org, double orient) 
{
    _xoffs = x_org;
    _yoffs = y_org;
    _cosA = cos(orient);
    _sinA = sin(orient);
}

void CoordFrame2D::GlobalToLocal(double &x, double &y) const
{
    double a = x - _xoffs;
    double b = y - _yoffs;
    x = a*_cosA + b*_sinA;
    y = -a*_sinA + b*_cosA; 
}

void CoordFrame2D::LocalToGlobal(double &x, double &y) const
{
    double a = x;
    double b = y;
    x = a*_cosA - b*_sinA + _xoffs;
    y = a*_sinA + b*_cosA + _yoffs;
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
        int degree)
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
        result += _coeffs[i] * i * pow(x, i-1);
    }

    return result;
}

// // Evaluate polynomial CppAD expression
// AD<double> Polynomial::Evaluate(const AD<double> &x) const
// {
//     size_t degree = _coeffs.size() - 1;
//     AD<double> result = _coeffs[0];

//     if (degree >= 1) {
//         result += AD<double>(x) * _coeffs[1];
//     }

//     for (size_t i = 2; i <= degree; i++) {
//         result += CppAD::pow(x, i) * _coeffs[i];
//     }

//     return result;
// }

// // Evaluate polynomial 1st derivative CppAD expression
// AD<double> Polynomial::Derivative(const AD<double> &x) const
// {
//     size_t degree = _coeffs.size() - 1;
//     AD<double> result = 0.0;

//     if (degree >= 1) {
//         result += _coeffs[1];
//     }
    
//     if (degree >= 2) {
//         result += AD<double>(x) * 2 * _coeffs[2];
//     }

//     for (size_t i = 3; i <= degree; i++) {
//         result += CppAD::pow(x, i-1) * i * _coeffs[i];
//     }

//     return result;
// }
