#include "xslam/opencv/geometry_transform/line_RANSAC.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace geometry_transform {

TEST(LineRANSAC, line)
{
    // OpenCV
    LineRANSAC demo;
    demo.RunDemo();
}

} // namespace geometry_transform 
} // namespace opencv
} // namespace xslam


