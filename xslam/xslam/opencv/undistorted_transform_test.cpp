//
// Created by quan on 2021/12/20.
//

#include "xslam/opencv/undistorted_transform.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

TEST(UndistortedTransform, UndistortMethod)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0005_distorted.png";
    UndistortedTransform demo;
    demo.RunDemo(filename);
}

} // namespace opencv
} // namespace xslam