#ifndef XSLAM_VINS_POSE_GRAPH_OPTIONS_H
#define XSLAM_VINS_POSE_GRAPH_OPTIONS_H

#include "xslam/vins/common/lua_parameter_dictionary.h"
#include "xslam/vins/pose_graph/proto/pose_graph_options.pb.h"

#include <thread>
#include <string>

namespace xslam {
namespace vins {
namespace pose_graph {
    
proto::PoseGraphOptions CreatePoseGraphOptions(
    common::LuaParameterDictionary* const lua_parameter_dictionary);

void VinsOptionsDebugToString(const proto::PoseGraphOptions& options);

} // namespace pose_graph
} // namespace vins
} // namespace xslam 

#endif // XSLAM_VINS_POSE_GRAPH_OPTIONS_H