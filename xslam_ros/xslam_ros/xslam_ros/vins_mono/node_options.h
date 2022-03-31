#ifndef XSLAM_ROS_VINS_MONO_NODE_OPTIONS_H
#define XSLAM_ROS_VINS_MONO_NODE_OPTIONS_H


#include <string>
#include <tuple>

#include "xslam/vins/estimator/proto/vins_builder_options.pb.h"

namespace xslam_ros {

// Top-level options of VINS-Mono's ROS integration.
struct NodeOptions 
{
    ::xslam::vins::estimator::proto::VinsBuilderOptions vins_builder_options;
    std::string map_frame;
    std::string tracking_frame;
    double lookup_transform_timeout_sec;
    double submap_publish_period_sec;
    double pose_publish_period_sec;
    double trajectory_publish_period_sec;
    bool publish_to_tf = true;
    bool publish_tracked_pose = false;
    bool use_pose_extrapolator = true;
};

// NodeOptions CreateNodeOptions(
//     ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

NodeOptions LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);

} // namespace xslam_ros

#endif // XSLAM_ROS_VINS_MONO_NODE_OPTIONS_H