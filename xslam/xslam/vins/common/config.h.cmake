#ifndef SLAM_COMMON_CONFIG_H
#define SLAM_COMMON_CONFIG_H

#include <string>

namespace xslam {
namespace common {

std::string kVinsSourceDirectory = "@PROJECT_SOURCE_DIR@";

constexpr char kConfigurationFilesDirectory[] =
    "@XSLAM_CONFIGURATION_FILES_DIRECTORY@";


}  // namespace common
}  // namespace xslam

#endif // URANUS_COMMON_CONFIG_H