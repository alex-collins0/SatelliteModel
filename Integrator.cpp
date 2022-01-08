#include <iostream>
#include <Eigen/Dense>
#include <Integrator.h>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

const double MAX_STEP_SIZE = 0.05;

std::tuple<double, VectorXd, double> integrate_Rkf(std::function<VectorXd(double, VectorXd)> rhs, double t, VectorXd y, double dt, double tol) {
    double a1 = 0.0;
    double a2 = 2.0/9.0;
    double a3 = 1.0/3.0;
    double a4 = 3.0/4.0;
    double a5 = 1.0;
    double a6 = 5.0/6.0;

    double b21 = 2.0/9.0;
    double b31 = 1.0/12.0;
    double b32 = 1.0/4.0;
    double b41 = 69.0/128.0;
    double b42 = -243.0/128.0;
    double b43 = 135.0/64.0;
    double b51 = -17.0/12.0;
    double b52 = 27.0/4.0;
    double b53 = -27.0/5.0;
    double b54 = 16.0/15.0;
    double b61 = 65.0/432.0;
    double b62 = -5.0/16.0;
    double b63 = 13.0/16.0;
    double b64 = 4.0/27.0;
    double b65 = 5.0/144.0;

    double ch1 = 47.0/450.0;
    double ch2 = 0.0;
    double ch3 = 12.0/25.0;
    double ch4 = 32.0/225.0;
    double ch5 = 1.0/30.0;
    double ch6 = 6.0/25.0;

    double ct1 = -1.0/150.0;
    double ct2 = 0.0;
    double ct3 = 3.0/100.0;
    double ct4 = -16.0/75.0;
    double ct5 = -1.0/20.0;
    double ct6 = 6.0/25.0;

    double truncation_error = tol + 1;
    double t_new;
    VectorXd y_new;

    while (truncation_error > tol) {

        VectorXd k1 = dt*rhs(t + a1*dt, y);
        VectorXd k2 = dt*rhs(t + a2*dt, y + b21*k1);
        VectorXd k3 = dt*rhs(t + a3*dt, y + b31*k1 + b32*k2);
        VectorXd k4 = dt*rhs(t + a4*dt, y + b41*k1 + b42*k2 + b43*k3);
        VectorXd k5 = dt*rhs(t + a5*dt, y + b51*k1 + b52*k2 + b53*k3 + b54*k4);
        VectorXd k6 = dt*rhs(t + a6*dt, y + b61*k1 + b62*k2 + b63*k3 + b64*k4 + b65*k5);

        t_new = t + dt;
        y_new = y + ch1 * k1 + ch2 * k2 + ch3 * k3 + ch4 * k4 + ch5 * k5 + ch6 * k6;

        truncation_error = (ct1 * k1 + ct2 * k2 + ct3 * k3 + ct4 * k4 + ct5 * k5 + ct6 * k6 ).norm();

        dt = 0.9 * dt * std::pow(tol / truncation_error, 0.2);

        dt = (dt > MAX_STEP_SIZE) ? MAX_STEP_SIZE : dt; // Make sure our step size isn't tooo big
    }

    return {t_new, y_new, dt};
}