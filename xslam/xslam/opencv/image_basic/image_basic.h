//
// Created by quan on 2021/12/20.
//

#ifndef SLAM_IMAGE_BASIC_H
#define SLAM_IMAGE_BASIC_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <string>

namespace xslam {
namespace opencv {

class ImageBasic
{
public:
    void RunDemo(const std::string& filename);
};

} // namespace opencv
} // namespace xslam

#endif //SLAM_IMAGE_BASIC_H
