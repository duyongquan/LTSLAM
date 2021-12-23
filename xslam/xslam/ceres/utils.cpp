//
// Created by quan on 2021/12/9.
//

#include "xslam/ceres/utils.h"
#include "xslam/common/config.h"

namespace xslam {
namespace ceres {

std::string GetCeresDatasetDirectory()
{
    return common::kSourceDirectory + "/data/ceres/";
}

} // namespace ceres
} // namespace xslam
