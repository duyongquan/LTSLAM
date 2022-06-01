#include "xslam/pangolin/hello_pangolin.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace pangolin {

TEST(HelloPangolin, hello)
{
    HelloPangolin demo;
    demo.RunDemo();
}

} // namespace pangolin
} // namespace xslam
