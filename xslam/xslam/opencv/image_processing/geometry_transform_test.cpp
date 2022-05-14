#include "xslam/opencv/image_processing/geometry_transform.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"


namespace xslam {
namespace opencv {
namespace image_processing {

TEST(GeometryTransform, demo)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0016_dog.jpg";
    GeometryTransform demo;
    demo.RunDemo(filename);
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
