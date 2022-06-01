#include "xslam/pangolin/simple_display.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace pangolin {

TEST(SimpleDisplay, hello)
{
    SimpleDisplay demo;
    demo.RunDemo();
}

} // namespace pangolin
} // namespace xslam
