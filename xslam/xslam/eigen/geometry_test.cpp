//
// Created by quan on 2021/12/20.
//

#include "gtest/gtest.h"
#include "glog/logging.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * 本程序演示了 Eigen 几何模块的使用方法
 */
namespace slam {
namespace eigen {

TEST(Eigen, RotationMatrix2Euler)
{
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX顺序，即yaw-pitch-roll顺序
    std::cout << "yaw pitch roll = "
              << euler_angles.transpose() << std::endl;
}

} // namespace opencv
} // namespace slam
