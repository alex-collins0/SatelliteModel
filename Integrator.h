#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

std::tuple<double, VectorXd, double> integrate_Rkf(std::function<VectorXd(double, VectorXd)>, double, VectorXd, double, double=1e-6);

#endif