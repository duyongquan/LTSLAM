#ifndef XSLAM_ROS_VINS_MONO_VINS_BUILDER_BRIDGE_H
#define XSLAM_ROS_VINS_MONO_VINS_BUILDER_BRIDGE_H

#include "xslam/vins/vins_builder.h"
#include "xslam_ros/vins_mono/sensor_bridge.h"
#include "xslam_ros/vins_mono/node_options.h"

#include <vector>
#include <mutex>
#include <memory>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace xslam_ros {
namespace vins_mono {

class VinsBuilderBridge 
{
public:
     VinsBuilderBridge(
        const NodeOptions& node_options,
        std::shared_ptr<xslam::vins::VinsBuilderInterface> vins_builder,
        tf2_ros::Buffer* tf_buffer);


    VinsBuilderBridge(const VinsBuilderBridge&) = delete;
    VinsBuilderBridge& operator=(const VinsBuilderBridge&) = delete;

    SensorBridge* sensor_bridge();

private:
    std::mutex mutex_;
    const NodeOptions node_options_;
    tf2_ros::Buffer* const tf_buffer_;

    std::unique_ptr<SensorBridge> sensor_bridges_;
};

}  // namespace vins_mono
}  // namespace xslam_ros

#endif  // XSLAM_ROS_VINS_MONO_VINS_BUILDER_BRIDGE_H
