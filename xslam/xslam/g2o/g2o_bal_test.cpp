#include "xslam/g2o/g2o_bal.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {


TEST(LargeBA, demo)
{
    LOG(INFO) << "Run G2O LargeBA.";
    const std::string filename = opencv::GetG2ODatasetDirectory() + "problem-16-22106-pre.txt";
    LargeBA demo;
    demo.RunDemo(filename);
}


} // namespace g2o
} // namespace xslam
