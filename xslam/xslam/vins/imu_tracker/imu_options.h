#ifndef XSLAM_VINS_IMU_TRACKER_IMU_TRACKER_OPTIONS_H
#define XSLAM_VINS_IMU_TRACKER_IMU_TRACKER_OPTIONS_H

#include "xslam/vins/common/lua_parameter_dictionary.h"
#include "xslam/vins/imu_tracker/proto/imu_tracker_options.pb.h"

namespace xslam {
namespace vins {
namespace imu_tracker {

proto::ImuTrackerOptions CreateImuTrackerOptions(
    common::LuaParameterDictionary* const lua_parameter_dictionary);

void OptionsDebugToString(const proto::ImuTrackerOptions& options);

} // namespace imu_tracker
} // namespace vins
} // namespace xslam 

#endif // XSLAM_VINS_IMU_TRACKER_IMU_TRACKER_OPTIONS_H
