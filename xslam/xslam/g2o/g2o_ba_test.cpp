#include "xslam/g2o/g2o_ba.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace g2o {
    
// Reference 
// https://www.cnblogs.com/gaoxiang12/p/5304272.html

TEST(SimpleBA, demo)
{
    LOG(INFO) << "Run G2O SimpleBA.";
    std::string image1 = opencv::GetG2ODatasetDirectory() + "001_simple_ba.png";
    std::string image2 = opencv::GetG2ODatasetDirectory() + "002_simple_ba.png";

    SimpleBA demo;
    demo.RunDemo(image1, image2);
}

} // namespace g2o
} // namespace xslam