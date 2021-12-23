//
// Created by quan on 2021/12/15.
//

#include "xslam/opencv/KLT.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

TEST(OpticalFlowPyrLK, follow)
{
    // 0001_fan_ren_xiu_xian_zhuan.flv
    // 0002_dota2.avi
    std::string filename = GetOpenCVDatasetDirectory() + "/0001_fan_ren_xiu_xian_zhuan.flv";
    OpticalFlowPyrLK demo;
    demo.Run(filename);
}

} // namespace opencv
} // namespace xslam