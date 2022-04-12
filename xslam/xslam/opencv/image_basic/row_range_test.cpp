#include "xslam/opencv/image_basic/row_range.h"

#include "xslam/opencv/utils.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

namespace xslam {
namespace opencv {
namespace image_basic {

TEST(RowColRange, RunRowRangeDemo)
{
    RowColRange demo;
    demo.RunRowRangeDemo();
}

TEST(RowColRange, RunColRangeDemo)
{
    RowColRange demo;
    demo.RunColRangeDemo();
}

} // namespace image_basic
} // namespace opencv
} // namespace xslam