#include "xslam/opencv/geometry_transform/epnp_eigen.h"
#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace geometry_transform {

TEST(EPnPEigen, epnp_eigen)
{
    std::string image = GetOpenCVDatasetDirectory() + "/box.xyz";
//     EPnPEigen demo;
//     demo.RunDemo(image);
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam