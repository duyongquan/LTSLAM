//
// Created by quan on 2021/12/14.
//

#ifndef SLAM_READ_IMAGE_H
#define SLAM_READ_IMAGE_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

class ReadImage
{
public:
    void Read(const std::string& filename);
};

} // namespace opencv
} // namespace xslam

// 002_chessboard.jpeg

#endif //SLAM_READ_IMAGE_H
