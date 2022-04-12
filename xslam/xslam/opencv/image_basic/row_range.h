#ifndef XSLAM_OPENCV_IMAGE_BASIC_ROW_RANGE_H
#define XSLAM_OPENCV_IMAGE_BASIC_ROW_RANGE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <boost/dynamic_bitset.hpp>

namespace xslam {
namespace opencv {
namespace image_basic {

class RowColRange
{
public:
    void RunDemo(const std::string& filename);

    // rowRange
    void RunRowRangeDemo();

    // colRange
    void RunColRangeDemo();

};

} // namespace image_basic
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_IMAGE_BASIC_ROW_RANGE_H
