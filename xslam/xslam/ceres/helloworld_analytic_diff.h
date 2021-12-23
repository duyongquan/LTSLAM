//
// Created by quan on 2021/12/7.
//

#ifndef SLAM_HELLOWORLD_ANALYTIC_DIFF_H
#define SLAM_HELLOWORLD_ANALYTIC_DIFF_H

#include "ceres/ceres.h"
#include "glog/logging.h"

namespace xslam {
namespace ceres {

// A CostFunction implementing analytically derivatives for the
// function f(x) = 10 - x.
class QuadraticCostFunction : public ::ceres::SizedCostFunction<1 /* number of residuals */,
                                                       1 /* size of first parameter */>
{
public:
    virtual ~QuadraticCostFunction() {}
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {
        double x = parameters[0][0];
        // f(x) = 10 - x.
        residuals[0] = 10 - x;

        // f'(x) = -1. Since there's only 1 parameter and that parameter
        // has 1 dimension, there is only 1 element to fill in the
        // jacobians.
        //
        // Since the Evaluate function can be called with the jacobians
        // pointer equal to NULL, the Evaluate function must check to see
        // if jacobians need to be computed.
        //
        // For this simple problem it is overkill to check if jacobians[0]
        // is NULL, but in general when writing more complex
        // CostFunctions, it is possible that Ceres may only demand the
        // derivatives w.r.t. a subset of the parameter blocks.
        if (jacobians != NULL && jacobians[0] != NULL)
        {
            jacobians[0][0] = -1;
        }
        return true;
    }
};


class AnalyticDiff
{
public:
    void RunDemo();
};

}
}


#endif //SLAM_HELLOWORLD_ANALYTIC_DIFF_H
