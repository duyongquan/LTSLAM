//
// Created by quan on 2021/12/14.
//

#include "xslam/common/config.h"
#include "xslam/opencv/utils.h"

namespace xslam {
namespace opencv {

std::string GetOpenCVDatasetDirectory()
{
    return common::kSourceDirectory + "/data/opencv/";
}

std::string GetG2ODatasetDirectory()
{
    return common::kSourceDirectory + "/data/g2o/";
}

} // namespace opencv
} // namespace xslam
