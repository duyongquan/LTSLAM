#include "xslam/opencv/image_processing/convexHull.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace image_processing {

TEST(ConvexHull, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0022_convex_hull.jpeg";
    ConvexHull demo;
    demo.RunDemo(filename);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
