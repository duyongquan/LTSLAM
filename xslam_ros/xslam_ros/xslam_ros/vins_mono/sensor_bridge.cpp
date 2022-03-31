#include "xslam_ros/vins_mono/sensor_bridge.h"

#include "xslam_ros/vins_mono/msg_conversion.h"
#include "xslam_ros/vins_mono/time_conversion.h"

namespace xslam_ros {
namespace vins_mono {

void SensorBridge::HandleImuMessage(
    const std::string& sensor_id,
    const sensor_msgs::Imu::ConstPtr& msg)
{

}


void SensorBridge::HandleImageMessage(
    const std::string& sensor_id,
    const sensor_msgs::Image::ConstPtr& msg)
{

}

std::unique_ptr<::xslam::vins::sensor::ImuData> SensorBridge::ToImuData(
        const sensor_msgs::Imu::ConstPtr& msg)
{
    const ::xslam::vins::common::Time time = FromRos(msg->header.stamp);

    return std::make_unique<::xslam::vins::sensor::ImuData>(
        ::xslam::vins::sensor::ImuData{
            time, 
            ToEigen(msg->linear_acceleration),
            ToEigen(msg->angular_velocity)
    });
}

// std::unique_ptr<::xslam::vins::sensor::ImageData> SensorBridge::ToImageData(
//         const sensor_msgs::Image::ConstPtr& msg)
// {
//      const ::xslam::vins::common::Time time = FromRos(msg->header.stamp);

//      return std::make_unique<::xslam::vins::sensor::ImageData>(::xslam::vins::sensor::ImuData{
//         time, 
//         ToEigen(msg->linear_acceleration),
//         ToEigen(msg->angular_velocity)
//     });
// }

}  // namespace vins_mono
}  // namespace xslam_ros