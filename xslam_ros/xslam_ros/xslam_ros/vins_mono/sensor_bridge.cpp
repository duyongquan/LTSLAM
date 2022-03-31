#include "xslam_ros/vins_mono/sensor_bridge.h"

#include "xslam_ros/vins_mono/msg_conversion.h"
#include "xslam_ros/vins_mono/time_conversion.h"

#include "glog/logging.h"

namespace xslam_ros {
namespace vins_mono {

SensorBridge::SensorBridge(
    const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, tf2_ros::Buffer* const tf_buffer,
    ::xslam::vins::VinsBuilderInterface*  vins_builder)
    : tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      vins_builder_(vins_builder) {}

void SensorBridge::HandleImuMessage(
    const std::string& sensor_id,
    const std::unique_ptr<::xslam::vins::sensor::ImuData>& msg)
{

  LOG(INFO) << "**********************";
    if (msg != nullptr) {
        vins_builder_->AddSensorData(sensor_id, 
            ::xslam::vins::sensor::ImuData {
                msg->time, 
                msg->linear_acceleration, 
                msg->angular_velocity
            });
    }
}

void SensorBridge::HandleImageMessage(
    const std::string& sensor_id,
    const std::unique_ptr<::xslam::vins::sensor::ImageData>&  msg)
{
    if (msg != nullptr) {
        vins_builder_->AddSensorData(sensor_id,
            ::xslam::vins::sensor::ImageData{msg->time, msg->image});
    }
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

std::unique_ptr<::xslam::vins::sensor::ImageData> SensorBridge::ToImageData(
      const sensor_msgs::Image::ConstPtr& msg)
{
    const ::xslam::vins::common::Time time = FromRos(msg->header.stamp);

    cv_bridge::CvImageConstPtr ptr;
    if (msg->encoding == "8UC1")
    {
        sensor_msgs::Image ros_image;
        ros_image.header = msg->header;
        ros_image.height = msg->height;
        ros_image.width  = msg->width;
        ros_image.is_bigendian = msg->is_bigendian;
        ros_image.step = msg->step;
        ros_image.data = msg->data;
        ros_image.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }

    ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    return std::make_unique<::xslam::vins::sensor::ImageData>(
        ::xslam::vins::sensor::ImageData{
            time, 
            ptr->image
    });
}

}  // namespace vins_mono
}  // namespace xslam_ros