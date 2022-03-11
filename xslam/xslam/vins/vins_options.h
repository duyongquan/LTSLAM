#ifndef XSLAM_VINS_VINS_OPTIONS_H
#define XSLAM_VINS_VINS_OPTIONS_H

#include <string>
#include <tuple>

#include "xslam/vins/common/lua_parameter_dictionary.h"

namespace xslam {
namespace vins {

struct VINSOptions {
    std::string tracking_frame;
    std::string filestem;
    double resolution;
};

VINSOptions CreateVinsOptions(common::LuaParameterDictionary* const lua_parameter_dictionary);

VINSOptions LoadOptions(const std::string& configuration_directory, const std::string& configuration_basename);

void VinsOptionsDebugToString(const VINSOptions& options);

} // namespace vins
} // namespace xslam

#endif // XSLAM_VINS_VINS_OPTIONS_H

