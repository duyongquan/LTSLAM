#include "xslam/libQGLViewer/multi_view.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace libQGLViewer {

TEST(MultiViewer, demo)
{
    MultiViewer demo;
    demo.RunDemo();
}

} // namespace libQGLViewer
} // namespace xslam