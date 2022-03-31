#ifndef XSLAM_ROS_VINS_MONO_SENSOR_BRIDGE_H
#define XSLAM_ROS_VINS_MONO_SENSOR_BRIDGE_H

#include "xslam/vins/vins_builder.h"
#include "xslam/vins/sensor/imu_data.h"
#include "xslam/vins/sensor/image_data.h"
#include "xslam_ros/vins_mono/node_options.h"

#include <vector>
#include <mutex>
#include <memory>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"

namespace xslam_ros {
namespace vins_mono {

class SensorBridge 
{
public:

    SensorBridge(const SensorBridge&) = delete;
    SensorBridge& operator=(const SensorBridge&) = delete;

    void HandleImuMessage(const std::string& sensor_id,
                          const sensor_msgs::Imu::ConstPtr& msg);

    void HandleImageMessage(const std::string& sensor_id,
                            const sensor_msgs::Image::ConstPtr& msg);

    std::unique_ptr<::xslam::vins::sensor::ImuData> ToImuData(
        const sensor_msgs::Imu::ConstPtr& msg);

    std::unique_ptr<::xslam::vins::sensor::ImageData> ToImageData(
        const sensor_msgs::Image::ConstPtr& msg);

private:
  
};

}  // namespace vins_mono
}  // namespace xslam_ros

#endif  // XSLAM_ROS_VINS_MONO_SENSOR_BRIDGE_H
