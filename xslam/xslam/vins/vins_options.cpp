#include "xslam/vins/vins_options.h"
#include "xslam/vins/common/configuration_file_resolver.h"

#include <memory>
#include <vector>

#include "glog/logging.h"

namespace xslam {
namespace vins {


VINSOptions CreateVinsOptions(common::LuaParameterDictionary* const lua_parameter_dictionary) 
{
    VINSOptions options;
    options.tracking_frame  = lua_parameter_dictionary->GetString("tracking_frame");
    options.filestem   = lua_parameter_dictionary->GetString("filestem");
    options.resolution = lua_parameter_dictionary->GetDouble("resolution");

    // Debug 
    VinsOptionsDebugToString(options);
    return options;
}

VINSOptions LoadOptions(
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

void VinsOptionsDebugToString(const VINSOptions& options)
{
    LOG(INFO) << "tracking_frame = " << options.tracking_frame;
    LOG(INFO) << "filestem = " << options.filestem;
    LOG(INFO) << "resolution = " << options.resolution;
}


} // namespace vins
} // namespace xslam