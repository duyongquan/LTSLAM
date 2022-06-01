#include "xslam/pangolin/simple_plot.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace pangolin {

TEST(SimplePlot, hello)
{
    SimplePlot demo;
    demo.RunDemo();
}

} // namespace pangolin
} // namespace xslam
