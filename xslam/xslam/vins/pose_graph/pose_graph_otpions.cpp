#include "xslam/vins/pose_graph/pose_graph_otpions.h"

namespace xslam {
namespace vins {
namespace pose_graph {
    

proto::PoseGraphOptions CreatePoseGraphOptions(
    common::LuaParameterDictionary* const lua_parameter_dictionary)
{
    proto::PoseGraphOptions options;

    options.set_max_solver_time(lua_parameter_dictionary->GetDouble("max_solver_time"));
    options.set_max_num_iterations(lua_parameter_dictionary->GetInt("max_num_iterations"));
    options.set_keyframe_parallax(lua_parameter_dictionary->GetDouble("keyframe_parallax"));
    // loop closure parameters
    options.set_loop_closure(lua_parameter_dictionary->GetInt("loop_closure"));
    options.set_load_previous_pose_graph(lua_parameter_dictionary->GetInt("load_previous_pose_graph"));
    options.set_fast_relocalization(lua_parameter_dictionary->GetInt("fast_relocalization"));
    return options;
}

void VinsOptionsDebugToString(const proto::PoseGraphOptions& options)
{

}


} // namespace pose_graph
} // namespace vins
} // namespace xslam 
