#include "xslam/opencv/geometry_transform/line_fitting_ransac.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace geometry_transform {

TEST(LineFittingRANSAC, line_fitting_ransac)
{
    LineFittingRANSAC demo;
    demo.RunDemo("");
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam