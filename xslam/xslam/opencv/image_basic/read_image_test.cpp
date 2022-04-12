//
// Created by quan on 2021/12/14.
//

#include "xslam/opencv/image_basic/read_image.h"
#include "xslam/opencv/utils.h"
#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

TEST(ReadImage, read_image_test)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0001_tian_an_man.jpeg";
    ReadImage demo;
    demo.Read(filename);
}

} // namespace opencv
} // namespace xslam