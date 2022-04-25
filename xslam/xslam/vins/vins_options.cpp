#include "xslam/vins/vins_options.h"
#include "xslam/vins/imu_tracker/imu_options.h"
#include "xslam/vins/pose_graph/pose_graph_otpions.h"
#include "xslam/vins/feature_tracker/feature_tracker_options.h"
#include "xslam/vins/common/configuration_file_resolver.h"

#include <memory>
#include <vector>

#include "glog/logging.h"

namespace xslam {
namespace vins {


estimator::proto::EstimatorOptions CreateVinsOptions(common::LuaParameterDictionary* const lua_parameter_dictionary) 
{
    estimator::proto::EstimatorOptions options;

    *options.mutable_imu_tracker_options() = imu_tracker::CreateImuTrackerOptions(
        lua_parameter_dictionary->GetDictionary("imu_tracker").get());
    // *options.mutable_feature_tracker_options() = feature_tracker::CreateFeatureTrackerOptions(
    //     lua_parameter_dictionary->GetDictionary("feature_tracker").get());
    // *options.mutable_pose_graph_options() = pose_graph::CreatePoseGraphOptions(
    //     lua_parameter_dictionary->GetDictionary("pose_graph").get());
    // options.set_thread_nums(lua_parameter_dictionary->GetInt("thread_nums"));
    return options;
}

estimator::proto::EstimatorOptions LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) 
{
    auto file_resolver =
        std::make_unique<common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});
    const std::string code = 
    file_resolver->GetFileContentOrDie(configuration_basename);
      common::LuaParameterDictionary lua_parameter_dictionary(
        code, std::move(file_resolver));

    return CreateVinsOptions(&lua_parameter_dictionary);
}

} // namespace vins
} // namespace xslam