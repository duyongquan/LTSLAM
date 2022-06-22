#include "xslam/opencv/3d_reconstruction/sfm_inc.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace reconstruction {

TEST(SFMInc, sfm_inc)
{
    std::string avi = GetOpenCVDatasetDirectory() + "/relief/%02d.jpg";
    SFMInc demo;
    demo.RunDemo(avi);
}

} // namespace reconstruction
} // namespace opencv
} // namespace xslam
