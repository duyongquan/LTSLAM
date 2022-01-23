#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "glog/logging.h"
#include "gtest/gtest.h"

#include <iostream>

#define MATRIX_SIZE 50

namespace slam {
namespace eigen {

TEST(Quaternion, demo)
{

    
    Eigen::Quaterniond q1(0.368668, 0.528906, 0.630209, -0.432642);
    Eigen::Quaterniond q2(0.350834, 0.505299, 0.649129, -0.44746);

    Eigen::Quaterniond result(q1.inverse() * q2);

    // result.w();
    
    // result.y();
    // result.z();

    std::cout  << "[" << result.w()
               << ", " << result.x()
               << ", " << result.y()
               << ", " << result.z() << " ]" << std::endl;

    Eigen::Matrix3d matrix(result);

    std::cout << "R = " << matrix << std::endl;

}

} // namespace
}