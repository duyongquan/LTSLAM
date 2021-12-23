//
// Created by quan on 2021/12/20.
//

#include "xslam/opencv/stereo_vision.h"


#include "xslam/opencv/utils.h"
#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

TEST(StereoVision, UndistortMethod)
{
    std::string image_filename_left  = GetOpenCVDatasetDirectory() + "/0007_stereo_left.png";
    std::string image_filename_right = GetOpenCVDatasetDirectory() + "/0007_stereo_right.png";

    StereoVision demo;
    demo.RunDemo(image_filename_left, image_filename_right);
}

} // namespace opencv
} // namespace xslam