#include "xslam_ros/vins_mono/vins_builder_bridge.h"

namespace xslam_ros {
namespace vins_mono {

VinsBuilderBridge::VinsBuilderBridge(
      const NodeOptions& node_options,
      std::unique_ptr<xslam::vins::VinsBuilderInterface> vins_builder,
      tf2_ros::Buffer* tf_buffer)
      : node_options_(node_options),
      vins_builder_(std::move(vins_builder)),
      tf_buffer_(tf_buffer) {}

SensorBridge* VinsBuilderBridge::sensor_bridge()
{
    return sensor_bridges_.get();
}

}  // namespace vins_mono
}  // namespace xslam_ros
