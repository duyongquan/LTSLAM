#include "xslam/sophus/basic.h"

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"


/*
namespace xslam {
namespace sophus {

void SophusMethod::RunDemo()
{
    // 沿Z轴转90度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

    // 或者四元数
    Eigen::Quaterniond q(R);
    Sophus::SO3d SO3_R(R);              // Sophus::SO3d可以直接从旋转矩阵构造
    Sophus::SO3d SO3_q(q);              // 也可以通过四元数构造

    // 二者是等价的
    std::cout << "SO(3) from matrix:\n" << SO3_R.matrix() << std::endl;
    std::cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << std::endl;
    std::cout << "they are equal" << std::endl;

    // 使用对数映射获得它的李代数
    Eigen::Vector3d so3 = SO3_R.log();
    std::cout << "so3 = " << so3.transpose() << std::endl;

    // hat 为向量到反对称矩阵
    std::cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << std::endl;

    // 相对的，vee为反对称到向量
    std::cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << std::endl;

    // 增量扰动模型的更新
    Eigen::Vector3d update_so3(1e-4, 0, 0); //假设更新量为这么多
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    std::cout << "SO3 updated = \n" << SO3_updated.matrix() << std::endl;

    std::cout << "*******************************" << std::endl;
    // 对SE(3)操作大同小异
    Eigen::Vector3d t(1, 0, 0);           // 沿X轴平移1
    Sophus::SE3d SE3_Rt(R, t);           // 从R,t构造SE(3)
    Sophus::SE3d SE3_qt(q, t);            // 从q,t构造SE(3)
    std::cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << std::endl;
    std::cout << "SE3 from q,t= \n" << SE3_qt.matrix() << std::endl;

    // 李代数se(3) 是一个六维向量，方便起见先typedef一下
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    std::cout << "se3 = " << se3.transpose() << std::endl;

    // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
    // 同样的，有hat和vee两个算符
    std::cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << std::endl;
    std::cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << std::endl;

    // 最后，演示一下更新
    Vector6d update_se3; //更新量
    update_se3.setZero();
    update_se3(0, 0) = 1e-4d;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    std::cout << "SE3 updated = " << std::endl << SE3_updated.matrix() << std::endl;
}

} // namespace sophus
} // namespace xslam

*/