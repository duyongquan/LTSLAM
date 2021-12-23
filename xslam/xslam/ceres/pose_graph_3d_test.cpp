//
// Created by quan on 2021/12/10.
//

#include "xslam/ceres/pose_graph_3d.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

namespace xslam {
namespace ceres {
namespace example {

// Python command run result :
// pose_graph_3d_plot_results.py --optimized_poses ./poses_optimized_3d.txt --initial_poses ./poses_original_3d.txt
TEST(PoseGraph3D, slam)
{
    LOG(INFO) << "Start PoseGraph3D .... ";
    PoseGraph3D demo;
    demo.RunSLAM();
}

} // namespace example
} // namespace ceres
} // namespace xslam