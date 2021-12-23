//
// Created by quan on 2021/12/20.
//

#include "xslam/ceres/quaternion_parameterization.h"
#include "Eigen/Geometry"

namespace xslam {
namespace ceres {

// https://blog.csdn.net/hzwwpgmwy/article/details/86490556
bool QuaternionParameterization::ComputeJacobian(const double* x, double* jacobian) const
{
    jacobian[0] = -x[1]; jacobian[1]  = -x[2]; jacobian[2]  = -x[3];
    jacobian[3] =  x[0]; jacobian[4]  =  x[3]; jacobian[5]  = -x[2];
    jacobian[6] = -x[3]; jacobian[7]  =  x[0]; jacobian[8]  =  x[1];
    jacobian[9] =  x[2]; jacobian[10] = -x[1]; jacobian[11] =  x[0];
    return true;
}

bool QuaternionParameterization::Plus(const double* x_ptr,
                                      const double* delta,
                                      double* x_plus_delta_ptr) const {
    Eigen::Map<Eigen::Quaterniond> x_plus_delta(x_plus_delta_ptr);
    Eigen::Map<const Eigen::Quaterniond> x(x_ptr);

    const double norm_delta = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);

    if (norm_delta > 0.0)
    {
        const double sin_delta_by_delta = sin(norm_delta) / norm_delta;

        // Note, in the constructor w is first.
        Eigen::Quaterniond delta_q(
                cos(norm_delta),
                sin_delta_by_delta * delta[0],
                sin_delta_by_delta * delta[1],
                sin_delta_by_delta * delta[2]);
        x_plus_delta = delta_q * x;
    } else {
        x_plus_delta = x;
    }

    return true;
}

void QuaternionParameter::RunDemo()
{

}

} // namespace ceres
} // namespace xslam