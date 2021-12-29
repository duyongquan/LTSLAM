#include "xslam/dbow3/loop_closure_detect.h"
#include "xslam/common/common.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

#include <string>

namespace xslam {
namespace dbow3 {

TEST(LoopClosureDetect, detect)
{
    std::vector<std::string> paths {
        ".", //  vocabulary path
        common::GetDBoW3DatasetDirectory() // dataset path
    };
    
    LoopClosureDetect demo;
    demo.RunDemo(paths);
}

} // namespace dbow3
} // namespace xslam