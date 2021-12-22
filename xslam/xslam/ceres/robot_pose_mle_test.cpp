//
// Created by quan on 2021/12/9.
//

#include "xslam/ceres/robot_pose_mle.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace slam {
namespace ceres {

TEST(RobotPoseMLE, RobotMotion)
{
    LOG(INFO) << "Start RobotPoseMLE .... ";
    RobotPoseMLE demo;
    demo.RunDemo();
}

} // namespace ceres
} // namespace slam
