#ifndef XSLAM_ROS_VINS_MONO_NODE_CONSTANTS_H
#define XSLAM_ROS_VINS_MONO_NODE_CONSTANTS_H

#include <string>
#include <vector>

namespace xslam_ros {

constexpr char kImuTopic[] = "imu";
constexpr char kImageTopic[] = "image";

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

} // namespace xslam_ros

#endif // XSLAM_ROS_VINS_MONO_NODE_CONSTANTS_H