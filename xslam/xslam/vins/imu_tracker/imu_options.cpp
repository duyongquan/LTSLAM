#include "xslam/vins/imu_tracker/imu_options.h"

#include "glog/logging.h"

namespace xslam {
namespace vins {
namespace imu_tracker {

proto::ImuTrackerOptions CreateImuTrackerOptions(
    common::LuaParameterDictionary* const lua_parameter_dictionary)
{
    proto::ImuTrackerOptions options;
    options.set_acceleration_n(lua_parameter_dictionary->GetDouble("acc_n"));
    options.set_gyro_n(lua_parameter_dictionary->GetDouble("gyr_n"));
    options.set_gyro_w(lua_parameter_dictionary->GetDouble("gyr_w"));
    options.set_acceleration_w(lua_parameter_dictionary->GetDouble("acc_w"));
    options.set_constant_gravity(lua_parameter_dictionary->GetDouble("g_norm"));

    // Debug 
    OptionsDebugToString(options);
    return options;
}

void OptionsDebugToString(const proto::ImuTrackerOptions& options)
{
    LOG(INFO) << "[ImuTrackerOptions:]";
    LOG(INFO) << "    acc_n: " << options.acceleration_n();
    LOG(INFO) << "    acc_w: " << options.acceleration_w();
    LOG(INFO) << "    gyro_n: " << options.gyro_n();
    LOG(INFO) << "    gyro_w: " << options.gyro_w();
    LOG(INFO) << "    g_norm: " << options.constant_gravity();
}

} // namespace imu_tracker
} // namespace vins
} // namespace xslam 