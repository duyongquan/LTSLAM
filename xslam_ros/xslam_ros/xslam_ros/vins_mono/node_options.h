#ifndef XSLAM_ROS_VINS_MONO_NODE_OPTIONS_H
#define XSLAM_ROS_VINS_MONO_NODE_OPTIONS_H


#include <string>
#include <tuple>

#include "xslam/vins/estimator/proto/estimator_options.pb.h"
#include "xslam/vins/common/lua_parameter_dictionary.h"

namespace xslam_ros {

// Top-level options of VINS-Mono's ROS integration.
struct NodeOptions 
{
    ::xslam::vins::estimator::proto::EstimatorOptions vins_options;
    std::string map_frame;
    std::string tracking_frame;
    std::string published_frame;
    double lookup_transform_timeout_sec;
    double pose_publish_period_sec;
    bool publish_to_tf = true;
    bool publish_tracked_pose = false;
};

NodeOptions CreateNodeOptions(::xslam::vins::common::LuaParameterDictionary* lua_parameter_dictionary);

NodeOptions LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);

void PrintOptions(const NodeOptions& options);


} // namespace xslam_ros

#endif // XSLAM_ROS_VINS_MONO_NODE_OPTIONS_H