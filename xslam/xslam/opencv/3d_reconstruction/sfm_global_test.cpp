#include "xslam/opencv/3d_reconstruction/sfm_global.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace reconstruction {

TEST(SFMGlobal, sfm_global)
{
    std::string avi = GetOpenCVDatasetDirectory() + "/relief/%02d.jpg";
    SFMGlobal demo;
    demo.RunDemo(avi);
}

} // namespace reconstruction
} // namespace opencv
} // namespace xslam
