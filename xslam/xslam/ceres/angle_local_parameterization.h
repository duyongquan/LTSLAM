//
// Created by quan on 2021/12/10.
//

#ifndef SLAM_ANGLE_LOCAL_PARAMETERIZATION_H
#define SLAM_ANGLE_LOCAL_PARAMETERIZATION_H

#include "ceres/local_parameterization.h"
#include "xslam/ceres/normalize_angle.h"

namespace xslam {
namespace ceres {
namespace example {

// Defines a local parameterization for updating the angle to be constrained in
// [-pi to pi).
class AngleLocalParameterization
{
public:
    template <typename T>
    bool operator()(const T* theta_radians, const T* delta_theta_radians,
                    T* theta_radians_plus_delta) const
    {
        *theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);
        return true;
    }

    static ::ceres::LocalParameterization* Create()
    {
        return (new ::ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
    }
};

} // namespace example
} // namespace ceres
} // namespace xslam

#endif //SLAM_ANGLE_LOCAL_PARAMETERIZATION_H
