//
// Created by quan on 2021/12/14.
//

#ifndef SLAM_SHI_TOMASI_H
#define SLAM_SHI_TOMASI_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

class ShiTomasi
{
public:
    void CornerDetect(const std::string& filename);
};

} // namespace opencv
} // namespace xslam


#endif //SLAM_SHI_TOMASI_H
