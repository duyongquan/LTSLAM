#include "xslam_ros/vins_mono/vins_builder_bridge.h"

namespace xslam_ros {
namespace vins_mono {

VinsBuilderBridge::VinsBuilderBridge(
      const NodeOptions& node_options,
      std::shared_ptr<xslam::vins::VinsBuilder> vins_builder,
      tf2_ros::Buffer* tf_buffer)
      : node_options_(node_options),
        vins_builder_(vins_builder),
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

xslam::vins::VinsBuilder* VinsBuilderBridge::vins_builder()
{
    return vins_builder_.get();
}

}  // namespace vins_mono
}  // namespace xslam_ros
