#ifndef XSLAM_VINS_SENSOR_IMAGE_DATA_H
#define XSLAM_VINS_SENSOR_IMAGE_DATA_H

#include "xslam/vins/common/time.h"

#include <opencv2/opencv.hpp>

namespace xslam {
namespace vins {
namespace sensor {

struct ImageData 
{
    common::Time time;
    cv::Mat image;
};

}  // namespace sensor
}  // namespace vins 
}  // namespace xslam

#endif  // XSLAM_VINS_SENSOR_IMAGE_DATA_H