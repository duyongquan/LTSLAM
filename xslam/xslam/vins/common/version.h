#ifndef XSLAM_VINS_COMMON_VERSION_H
#define XSLAM_VINS_COMMON_VERSION_H

#include <string>

namespace xslam {
namespace vins {
namespace common {

const static std::string VINS_VERSION = "1.0";
const static int VINS_VERSION_NUMBER = 1000;
const static std::string VINS_COMMIT_ID = "6e10e3e";
const static std::string VINS_COMMIT_DATE = "2021-11-19";

std::string GetVersionInfo();

std::string GetBuildInfo();

}  // namespace common
}  // namespace vins
}  // namespace xslam

#endif // XSLAM_VINS_COMMON_VERSION_H