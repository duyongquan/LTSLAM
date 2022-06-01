#include "xslam/pangolin/simple_multi_display.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace pangolin {

TEST(SimpleMultiDisplay, hello)
{
    SimpleMultiDisplay demo;
    demo.RunDemo();
}

} // namespace pangolin
} // namespace xslam
