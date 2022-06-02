#include "xslam/libQGLViewer/simple_viewer.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace libQGLViewer {

TEST(SimpleViewer, demo)
{
    SimpleViewer demo;
    demo.RunDemo();
}

} // namespace libQGLViewer
} // namespace xslam