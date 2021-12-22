//
// Created by quan on 2021/12/14.
//

#include "xslam/opencv/utils.h"
#include "xslam/common/config.h"

namespace slam {
namespace opencv {

std::string GetOpenCVDatasetDirectory()
{
    return common::kSourceDirectory + "/data/opencv/";
}

} // namespace opencv
} // namespace slam
