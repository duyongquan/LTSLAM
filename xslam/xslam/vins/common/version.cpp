#include "xslam/vins/common/version.h"
#include "xslam/vins/common/string.h"

namespace xslam {
namespace vins {
namespace common {


std::string GetVersionInfo()
{
    return StringPrintf("VINS %s", VINS_VERSION.c_str());
}

std::string GetBuildInfo()
{
    return StringPrintf("Commit %s on %s", VINS_COMMIT_ID.c_str(),
        VINS_COMMIT_DATE.c_str());
}

}  // namespace common
}  // namespace vins
}  // namespace xslam
