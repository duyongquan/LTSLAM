#ifndef XSLAM_OPENCV_GEOMETRY_TRANSFORM_OBJECT_LOCALIZATION_H
#define XSLAM_OPENCV_GEOMETRY_TRANSFORM_OBJECT_LOCALIZATION_H

#include "opencv2/opencv.hpp"
#include "glog/logging.h"

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>

namespace xslam {
namespace opencv {
namespace geometry_transform {

#define DEG2RAD(v)  (v * CV_PI / 180)
#define Rx(rx)      (cv::Matx33d(1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx)))
#define Ry(ry)      (cv::Matx33d(cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry)))
#define Rz(rz)      (cv::Matx33d(cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1))

void MouseEventHandler(int event, int x, int y, int flags, void* param);

class MouseDrag
{
public:
    MouseDrag() : dragged(false) { }
    bool dragged;
    cv::Point start, end;
};

class ObjectLocalization
{
public:
    void RunDemo(const std::string& filename);
};

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_GEOMETRY_TRANSFORM_OBJECT_LOCALIZATION_H

