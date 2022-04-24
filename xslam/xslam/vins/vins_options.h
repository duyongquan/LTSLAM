#ifndef XSLAM_VINS_VINS_OPTIONS_H
#define XSLAM_VINS_VINS_OPTIONS_H

#include <string>
#include <tuple>

#include "xslam/vins/common/lua_parameter_dictionary.h"
#include "xslam/vins/estimator/proto/estimator_options.pb.h"

namespace xslam {
namespace vins {

estimator::proto::EstimatorOptions CreateVinsOptions(common::LuaParameterDictionary* const lua_parameter_dictionary);

estimator::proto::EstimatorOptions LoadOptions(const std::string& configuration_directory, const std::string& configuration_basename);

void VinsOptionsDebugToString(const estimator::proto::EstimatorOptions& options);

} // namespace vins
} // namespace xslam

#endif // XSLAM_VINS_VINS_OPTIONS_H

