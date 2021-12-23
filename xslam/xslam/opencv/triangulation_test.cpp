//
// Created by quan on 2021/12/20.
//

#include "xslam/opencv/triangulation.h"
#include "xslam/opencv/utils.h"
#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

TEST(Triangulation, triangulation)
{
    std::string image_filename_left  = GetOpenCVDatasetDirectory() + "/0007_stereo_left.png";
    std::string image_filename_right = GetOpenCVDatasetDirectory() + "/0007_stereo_right.png";

    Triangulation demo;
    demo.RunDemo(image_filename_left, image_filename_right);
}

} // namespace opencv
} // namespace xslam