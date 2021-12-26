#ifndef XSLAM_VINS_CAMERA_DATA_H
#define XSLAM_VINS_CAMERA_DATA_H

#include "xslam/vins/common/time.h"

#include <opencv2/opencv.hpp>

namespace xslam {
namespace sensor {

struct CameraData 
{
    common::Time time;
    cv::Mat image;
};

}  // namespace sensor
}  // namespace xslam

#endif  // XSLAM_VINS_CAMERA_DATA_H