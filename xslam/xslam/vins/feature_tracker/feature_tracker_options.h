#ifndef XSLAM_VINS_FEATURE_TRACKER_OPTIONS_H
#define XSLAM_VINS_FEATURE_TRACKER_OPTIONS_H

#include <string>

#include "xslam/vins/common/lua_parameter_dictionary.h"
#include "xslam/vins/feature_tracker/proto/feature_tracker_options.pb.h"

namespace xslam {
namespace vins {
namespace feature_tracker {

proto::FeatureTrackerOptions CreateFeatureTrackerOptions(
    common::LuaParameterDictionary* const lua_parameter_dictionary);

void VinsOptionsDebugToString(const proto::FeatureTrackerOptions& options);

} // namespace feature_tracker
} // namespace vins
} // namespace xslam 

#endif // XSLAM_VINS_FEATURE_TRACKER_OPTIONS_H