//
// Created by quan on 2021/12/7.
//

#ifndef SLAM_ROBUST_CURVE_FITTING_H
#define SLAM_ROBUST_CURVE_FITTING_H


#include "ceres/ceres.h"
#include "glog/logging.h"

namespace xslam {
namespace ceres {

struct ExponentialResidual
{
    ExponentialResidual(double x, double y)
            : x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T* const m, const T* const c, T* residual) const
    {
        residual[0] = y_ - exp(m[0] * x_ + c[0]);
        return true;
    }

private:
    const double x_;
    const double y_;
};

class RobustCurveFitting
{
public:
    void RunDemo();
};

} // namespace ceres
} // namespace xslam


#endif //SLAM_ROBUST_CURVE_FITTING_H
