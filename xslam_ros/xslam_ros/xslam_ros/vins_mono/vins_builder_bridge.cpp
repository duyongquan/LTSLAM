#include "xslam_ros/vins_mono/vins_builder_bridge.h"

namespace xslam_ros {
namespace vins_mono {

VinsBuilderBridge::VinsBuilderBridge(
      const NodeOptions& node_options,
      std::shared_ptr<xslam::vins::VinsBuilderInterface> vins_builder,
      tf2_ros::Buffer* tf_buffer)
      : node_options_(node_options),
      tf_buffer_(tf_buffer) 
{

     sensor_bridges_ = std::make_unique<SensorBridge>(
        node_options_.tracking_frame,
        node_options_.lookup_transform_timeout_sec, tf_buffer_,
        vins_builder.get());
}

SensorBridge* VinsBuilderBridge::sensor_bridge()
{
    return sensor_bridges_.get();
}

}  // namespace vins_mono
}  // namespace xslam_ros
