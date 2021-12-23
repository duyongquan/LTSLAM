//
// Created by quan on 2021/12/9.
//


#include "xslam/ceres/pose_graph_2d.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace xslam {
namespace ceres {
namespace example {


// Python command run result :
// pose_graph_2d_plot_results.py --optimized_poses ./poses_optimized.txt --initial_poses ./poses_original.txt
TEST(PoseGraph2D, slam)
{
    LOG(INFO) << "Start RobotPoseMLE .... ";
    PoseGraph2D demo;
    demo.RunSLAM();
}

} // namespace example
} // namespace ceres
} // namespace slam