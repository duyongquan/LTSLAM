//
// Created by quan on 2021/12/20.
//

#include "xslam/opencv/image_basic.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

TEST(ImageBasic, ImageOperator)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0006_ubuntu.png";
    ImageBasic demo;
    demo.RunDemo(filename);
}

} // namespace opencv
} // namespace xslam