//
// Created by quan on 2021/12/20.
//

#include "xslam/opencv/rgbd_map.h"

#include "xslam/opencv/utils.h"
#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
// 6DOF t1 t2 ...
TEST(RGBDMap, BuildMap)
{
    // std::string filename = GetOpenCVDatasetDirectory() + "/rgbd/pose.txt";
    // std::string color_depth_dir = GetOpenCVDatasetDirectory() + "/rgbd/";
    // RGBDMap demo;
    // demo.RunDemo(filename, color_depth_dir);
}

} // namespace opencv
} // namespace xslam