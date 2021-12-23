//
// Created by quan on 2021/12/20.
//

#ifndef SLAM_QUATERNION_PARAMETERIZATION_H
#define SLAM_QUATERNION_PARAMETERIZATION_H

#include "ceres/ceres.h"

namespace xslam {
namespace ceres {

class QuaternionParameterization : public ::ceres::LocalParameterization
{
public:
    virtual ~QuaternionParameterization() {}

    // 实现优化变量更新
    virtual bool Plus(const double* x_ptr,
                      const double* delta,
                      double* x_plus_delta_ptr) const;

    virtual bool ComputeJacobian(const double* x, double* jacobian) const;

    // 表示参数 x
    //x 的自由度（可能有冗余），比如四元数的自由度是4，旋转矩阵的自由度是9
    virtual int GlobalSize() const { return 4; }

    // 表示 Δx
    //Δx 所在的正切空间（tangent space）的自由度，那么这个自由度是3
    virtual int LocalSize() const { return 3; }
};


class QuaternionParameter
{
public:
    void RunDemo();
};

} // namespace ceres
} // namespace xslam

#endif //SLAM_QUATERNION_PARAMETERIZATION_H
