#include "xslam/vins/pose_graph/pose_graph_otpions.h"

#include "glog/logging.h"

namespace xslam {
namespace vins {
namespace pose_graph {
    

proto::PoseGraphOptions CreatePoseGraphOptions(
    common::LuaParameterDictionary* const lua_parameter_dictionary)
{
    proto::PoseGraphOptions options;
    // optimization parameters
    options.set_max_solver_time(lua_parameter_dictionary->GetDouble("max_solver_time"));
    options.set_max_num_iterations(lua_parameter_dictionary->GetInt("max_num_iterations"));
    options.set_keyframe_parallax(lua_parameter_dictionary->GetDouble("keyframe_parallax"));

    // loop closure parameters
    options.set_loop_closure(lua_parameter_dictionary->GetInt("loop_closure"));
    options.set_load_previous_pose_graph(lua_parameter_dictionary->GetInt("load_previous_pose_graph"));
    options.set_fast_relocalization(lua_parameter_dictionary->GetInt("fast_relocalization"));

    // Debug 
    OptionsDebugToString(options);
    return options;
}

void OptionsDebugToString(const proto::PoseGraphOptions& options)
{
    LOG(INFO) << "[PoseGraphOptions:]";
    // optimization parameters
    LOG(INFO) << "  optimization parameters:";
    LOG(INFO) << "      max_solver_time: " << options.max_solver_time();
    LOG(INFO) << "      max_num_iterations: " << options.max_num_iterations();
    LOG(INFO) << "      keyframe_parallax: " << options.keyframe_parallax();

    // loop closure parameters
    LOG(INFO) << "  loop closure parameters:";
    LOG(INFO) << "      loop_closure: " << options.loop_closure();
    LOG(INFO) << "      load_previous_pose_graph: " << options.load_previous_pose_graph();
    LOG(INFO) << "      fast_relocalization: " << options.fast_relocalization();
}


} // namespace pose_graph
} // namespace vins
} // namespace xslam 
