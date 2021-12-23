//
// Created by quan on 2021/12/7.
//

#ifndef SLAM_HELLOWORLD_NUMERIC_DIFF_H
#define SLAM_HELLOWORLD_NUMERIC_DIFF_H

#include "ceres/ceres.h"
#include "glog/logging.h"

// min f(x) = 10 - x
namespace xslam {
namespace ceres {

/*
struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};
*/

struct NumericDiffCostFunctor
{
    // x : input data
    // residual: 残差或者误差
    bool operator()(const double* const x, double* residual) const
    {
        residual[0] = 10.0 - x[0];
        return true;
    }
};


class NumericDiff
{
public:
    void RunDemo();
};

}
}

#endif //SLAM_HELLOWORLD_NUMERIC_DIFF_H
