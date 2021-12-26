#include "xslam/opencv/image_pyramid.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"


namespace xslam {
namespace opencv {

TEST(ImagePyramid, PyrDown)
{
    std::string filename = GetOpenCVDatasetDirectory() + "/0002_chessboard.jpeg";
    ImagePyramid demo;
    demo.PyrDown(filename, 5);
}

} // namespace opencv
} // namespace xslam
