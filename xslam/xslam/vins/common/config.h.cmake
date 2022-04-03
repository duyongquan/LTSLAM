#ifndef SLAM_COMMON_CONFIG_H
#define SLAM_COMMON_CONFIG_H

#include <string>

namespace xslam {
namespace vins {
namespace common {

static std::string kVinsSourceDirectory = "@PROJECT_SOURCE_DIR@";

static std::string  kConfigurationFilesDirectory =
    "@PROJECT_SOURCE_DIR@/xslam/vins/configuration";


}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif // URANUS_COMMON_CONFIG_H