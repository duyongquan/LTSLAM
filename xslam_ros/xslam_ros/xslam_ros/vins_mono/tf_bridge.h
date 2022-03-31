#ifndef XSLAM_ROS_VINS_MONO_TF_BRIDGE_H
#define XSLAM_ROS_VINS_MONO_TF_BRIDGE_H

#include <string>
#include <memory>
#include "xslam/vins/transform/transform.h"
#include "xslam_ros/vins_mono/time_conversion.h"
#include "tf2_ros/buffer.h"

namespace xslam_ros {
namespace vins_mono {

class TfBridge 
{
public:
    TfBridge(const std::string& tracking_frame,
             double lookup_transform_timeout_sec, const tf2_ros::Buffer* buffer);
    ~TfBridge() {}

    TfBridge(const TfBridge&) = delete;
    TfBridge& operator=(const TfBridge&) = delete;

    // Returns the transform for 'frame_id' to 'tracking_frame_' if it exists at 'time'.
    // std::unique_ptr<::xslam::vins:::transform::Rigid3d> LookupToTracking(
    //     ::xslam::vins::common::Time time, const std::string& frame_id) const;

private:
    const std::string tracking_frame_;
    const double lookup_transform_timeout_sec_;
    const tf2_ros::Buffer* const buffer_;
};


}  // namespace vins_mono
}  // namespace xslam_ros

#endif // XSLAM_ROS_VINS_MONO_TF_BRIDGE_H