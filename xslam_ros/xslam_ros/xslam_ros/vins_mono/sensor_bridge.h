#ifndef XSLAM_ROS_VINS_MONO_SENSOR_BRIDGE_H
#define XSLAM_ROS_VINS_MONO_SENSOR_BRIDGE_H

#include "xslam/vins/vins_builder.h"
#include "xslam/vins/sensor/imu_data.h"
#include "xslam/vins/sensor/image_data.h"
#include "xslam_ros/vins_mono/tf_bridge.h"
#include "xslam_ros/vins_mono/node_options.h"
#include "xslam/vins/vins_builder_interface.h"

#include <vector>
#include <mutex>
#include <memory>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace xslam_ros {
namespace vins_mono {

class SensorBridge 
{
public:
    explicit SensorBridge(
        const std::string& tracking_frame,
        double lookup_transform_timeout_sec, tf2_ros::Buffer* tf_buffer,
        ::xslam::vins::VinsBuilderInterface* vins_builder);

    SensorBridge(const SensorBridge&) = delete;
    SensorBridge& operator=(const SensorBridge&) = delete;

    void HandleImuMessage(const std::string& sensor_id,
                          const std::unique_ptr<::xslam::vins::sensor::ImuData>& msg);

    void HandleImageMessage(const std::string& sensor_id,
                            const std::unique_ptr<::xslam::vins::sensor::ImageData>& msg);

    std::unique_ptr<::xslam::vins::sensor::ImuData> ToImuData(
        const sensor_msgs::Imu::ConstPtr& msg);

    std::unique_ptr<::xslam::vins::sensor::ImageData> ToImageData(
        const sensor_msgs::Image::ConstPtr& msg);

private:
    const TfBridge tf_bridge_;
    ::xslam::vins::VinsBuilderInterface* const vins_builder_;
};

}  // namespace vins_mono
}  // namespace xslam_ros

#endif  // XSLAM_ROS_VINS_MONO_SENSOR_BRIDGE_H
