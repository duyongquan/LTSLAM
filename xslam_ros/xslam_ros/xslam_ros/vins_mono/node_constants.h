#ifndef XSLAM_ROS_VINS_MONO_NODE_CONSTANTS_H
#define XSLAM_ROS_VINS_MONO_NODE_CONSTANTS_H

#include <string>
#include <vector>

namespace xslam_ros {
namespace vins_mono {


constexpr char kImuTopic[] = "imu0";
constexpr char kImageTopic[] = "cam0/image_raw";

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

} // namespace vins_mono
} // namespace xslam_ros

#endif // XSLAM_ROS_VINS_MONO_NODE_CONSTANTS_H