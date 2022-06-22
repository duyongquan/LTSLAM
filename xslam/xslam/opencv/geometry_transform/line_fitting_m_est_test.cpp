#include "xslam/opencv/geometry_transform/line_fitting_m_est.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace geometry_transform {

TEST(LineFittingMEst, line_fitting_m_est)
{
    LineFittingMEst demo;
    demo.RunDemo("");
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam