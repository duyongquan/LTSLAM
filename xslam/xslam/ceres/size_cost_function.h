//
// Created by quan on 2021/12/20.
//

#ifndef SLAM_SIZE_COST_FUNCTION_H
#define SLAM_SIZE_COST_FUNCTION_H

#include "ceres/ceres.h"

namespace xslam {
namespace ceres {

class Rat43Analytic : public ::ceres::SizedCostFunction<1,4>
{
public:
    Rat43Analytic(const double x, const double y) : x_(x), y_(y) {}
    virtual ~Rat43Analytic() {}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {
        const double b1 = parameters[0][0];
        const double b2 = parameters[0][1];
        const double b3 = parameters[0][2];
        const double b4 = parameters[0][3];

        residuals[0] = b1 *  pow(1 + exp(b2 -  b3 * x_), -1.0 / b4) - y_;

        if (!jacobians) return true;
        double* jacobian = jacobians[0];
        if (!jacobian) return true;

        jacobian[0] = pow(1 + exp(b2 - b3 * x_), -1.0 / b4);
        jacobian[1] = -b1 * exp(b2 - b3 * x_) *
                      pow(1 + exp(b2 - b3 * x_), -1.0 / b4 - 1) / b4;
        jacobian[2] = x_ * b1 * exp(b2 - b3 * x_) *
                      pow(1 + exp(b2 - b3 * x_), -1.0 / b4 - 1) / b4;
        jacobian[3] = b1 * log(1 + exp(b2 - b3 * x_)) *
                      pow(1 + exp(b2 - b3 * x_), -1.0 / b4) / (b4 * b4);
        return true;
    }

private:
    const double x_;
    const double y_;
};


class Rat43AnalyticOptimized : public ::ceres::SizedCostFunction<1,4>
{
public:
    Rat43AnalyticOptimized(const double x, const double y) : x_(x), y_(y) {}
    virtual ~Rat43AnalyticOptimized() {}
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        const double b1 = parameters[0][0];
        const double b2 = parameters[0][1];
        const double b3 = parameters[0][2];
        const double b4 = parameters[0][3];

        const double t1 = exp(b2 -  b3 * x_);
        const double t2 = 1 + t1;
        const double t3 = pow(t2, -1.0 / b4);
        residuals[0] = b1 * t3 - y_;

        if (!jacobians) return true;
        double* jacobian = jacobians[0];
        if (!jacobian) return true;

        const double t4 = pow(t2, -1.0 / b4 - 1);
        jacobian[0] = t3;
        jacobian[1] = -b1 * t1 * t4 / b4;
        jacobian[2] = -x_ * jacobian[1];
        jacobian[3] = b1 * log(t2) * t3 / (b4 * b4);
        return true;
    }

private:
    const double x_;
    const double y_;
};

class SizeCostFunction
{
public:
    void RunDemo();
    void RunOptimizedDemo();
};

} // namespace ceres
} // namespace xslam

#endif //SLAM_SIZE_COST_FUNCTION_H
