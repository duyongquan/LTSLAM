#include "xslam_ros/vins_mono/tf_bridge.h"
#include "xslam_ros/vins_mono/msg_conversion.h"
#include "xslam_ros/vins_mono/time_conversion.h"

#include <memory>

namespace xslam_ros {
namespace vins_mono {

TfBridge::TfBridge(const std::string& tracking_frame,
                   const double lookup_transform_timeout_sec,
                   const tf2_ros::Buffer* buffer)
    : tracking_frame_(tracking_frame),
      lookup_transform_timeout_sec_(lookup_transform_timeout_sec),
      buffer_(buffer) {}


// std::unique_ptr<::xslam::vins:::transform::Rigid3d> TfBridge::LookupToTracking(
//     const ::xslam::vins::common::Time time,
//     const std::string& frame_id) const 
// {
//     ::ros::Duration timeout(lookup_transform_timeout_sec_);
//     std::unique_ptr<::xslam::vins::transform::Rigid3d> frame_id_to_tracking;
//     try {
//       const ::ros::Time latest_tf_time =
//           buffer_->lookupTransform(tracking_frame_, frame_id, ::ros::Time(0.), timeout).header.stamp;
//       const ::ros::Time requested_time = ToRos(time);
//       if (latest_tf_time >= requested_time) 
//       {
//           // We already have newer data, so we do not wait. Otherwise, we would wait
//           // for the full 'timeout' even if we ask for data that is too old.
//           timeout = ::ros::Duration(0.);
//       }
//       return std::make_unique<::xslam::vins::transform::Rigid3d>(
//           ToRigid3d(buffer_->lookupTransform(tracking_frame_, frame_id,
//                                             requested_time, timeout)));
//     } catch (const tf2::TransformException& ex) {
//       LOG(WARNING) << ex.what();
//     }
//     return nullptr;
// }

}  // namespace vins_mono
}  // namespace xslam_ros