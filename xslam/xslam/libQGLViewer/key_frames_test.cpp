#include "xslam/libQGLViewer/key_frames.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace libQGLViewer {

TEST(KeyFrames, demo)
{
    KeyFrames demo;
    demo.RunDemo();
}

} // namespace libQGLViewer
} // namespace xslam