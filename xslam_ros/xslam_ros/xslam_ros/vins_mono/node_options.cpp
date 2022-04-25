#include "xslam_ros/vins_mono/node_options.h"
#include "xslam/vins/vins_options.h"
#include "xslam/vins/common/configuration_file_resolver.h"

namespace xslam_ros {

NodeOptions LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename)
{
   auto file_resolver = 
      std::make_unique<::xslam::vins::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  const std::string code = file_resolver->GetFileContentOrDie(configuration_basename);

  ::xslam::vins::common::LuaParameterDictionary 
      lua_parameter_dictionary(code, std::move(file_resolver));

  return CreateNodeOptions(&lua_parameter_dictionary);
}

NodeOptions CreateNodeOptions(::xslam::vins::common::LuaParameterDictionary* lua_parameter_dictionary)
{
    NodeOptions options;
    options.vins_options = ::xslam::vins::CreateVinsOptions(
        lua_parameter_dictionary->GetDictionary("vins_options").get());

    options.map_frame = lua_parameter_dictionary->GetString("map_frame");
    options.tracking_frame = lua_parameter_dictionary->GetString("tracking_frame");
    options.published_frame = lua_parameter_dictionary->GetString("published_frame");

    options.lookup_transform_timeout_sec =
        lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");

    options.pose_publish_period_sec =
        lua_parameter_dictionary->GetDouble("pose_publish_period_sec");

  
    if (lua_parameter_dictionary->HasKey("publish_to_tf")) {
      options.publish_to_tf =
          lua_parameter_dictionary->GetBool("publish_to_tf");
    }

    if (lua_parameter_dictionary->HasKey("publish_tracked_pose")) {
      options.publish_tracked_pose =
          lua_parameter_dictionary->GetBool("publish_tracked_pose");
    }
 
    PrintOptions(options);
    return options;
}



void PrintOptions(const NodeOptions& options)
{
    LOG(INFO) << "map_frame : " << options.map_frame;
    LOG(INFO) << "tracking_frame : " << options.tracking_frame;
    LOG(INFO) << "published_frame : " << options.published_frame;
    LOG(INFO) << "pose_publish_period_sec : " << options.pose_publish_period_sec;
}

} // namespace xslam_ros