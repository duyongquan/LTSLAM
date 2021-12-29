#include "xslam/common/common.h"
#include "xslam/common/config.h"

namespace xslam {
namespace common {

std::string GetDBoW3DatasetDirectory()
{
    return common::kSourceDirectory + "/data/dbow3/data/";
}

} // namespace common
} // namespace xslam
