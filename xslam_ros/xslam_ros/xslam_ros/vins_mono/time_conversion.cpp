#include "xslam_ros/vins_mono/time_conversion.h"

namespace xslam_ros {
namespace vins_mono {

::ros::Time ToRos(::xslam::vins::common::Time time)
{
    int64_t uts_timestamp = ::xslam::vins::common::ToUniversal(time);
    int64_t ns_since_unix_epoch = (uts_timestamp -
        ::xslam::vins::common::kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
    ::ros::Time ros_time;
    ros_time.fromNSec(ns_since_unix_epoch);
    return ros_time;
}

::xslam::vins::common::Time FromRos(const ::ros::Time& time)
{
    // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
    // exactly 719162 days before the Unix epoch.
    return ::xslam::vins::common::FromUniversal(
        (time.sec +
        ::xslam::vins::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
            10000000ll +
        (time.nsec + 50) / 100);  // + 50 to get the rounding correct.
}

}  // namespace vins_mono
}  // namespace xslam_ros