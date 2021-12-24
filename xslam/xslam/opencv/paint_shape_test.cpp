#include "xslam/opencv/paint_shape.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {

TEST(PaintShape, shape)
{
    LOG(INFO) << "Run PaintShape demos ...";
    // OpenCV
    PaintShape demo;
    demo.RunDemo();
}

} // namespace opencv
} // namespace xslam
