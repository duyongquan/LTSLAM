#ifndef XSLAM_ROS_VINS_MONO_TIME_CONVERSION_H
#define XSLAM_ROS_VINS_MONO_TIME_CONVERSION_H

#include "xslam/vins/common/time.h"
#include "ros/ros.h"

namespace xslam_ros {
namespace vins_mono {

::ros::Time ToRos(::xslam::vins::common::Time time);

::xslam::vins::common::Time FromRos(const ::ros::Time& time);

}  // namespace vins_mono
}  // namespace xslam_ros

#endif  // XSLAM_ROS_VINS_MONO_TIME_CONVERSION_H
