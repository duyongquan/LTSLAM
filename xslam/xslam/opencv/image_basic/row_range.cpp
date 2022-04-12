#include "xslam/opencv/image_basic/row_range.h"

namespace xslam {
namespace opencv {
namespace image_basic {

// rowRange
// C++: Mat Mat::rowRange(int startrow, int endrow) 
// 为指定的行空间创建矩阵标头。

// 参数说明：
// startrow - 行空间包含基于0的起始索引。
// endrow - 一个基于0的行空间结束索引。

// colRange
// 为指定的列范围创建矩阵标头。

// C++: Mat Mat::colRange(int startcol, int endcol)
// 为指定的列范围创建矩阵标头。

// 参数说明：
// startcol - 列跨度的包含0的开始索引。
// endcol - 列跨度的从0开始的独占结束索引。

void RowColRange::RunDemo(const std::string& filename)
{

}


void RowColRange::RunRowRangeDemo()
{
    cv::Mat mat = (cv::Mat_<double>(3, 3) << 0, 1, 2, 3, 4, 5, 6, 7, 8);
    std::cout << "matrix: " << std::endl
              << mat << std::endl << std::endl;

    cv::Mat row = mat.rowRange(0, 2).clone();   // 包括左边界，但不包括右边界
    std::cout << "Row range:" << std::endl
              << row << std::endl;

    std::cout << "Test 1 row:" << std::endl 
              << row.row(0) << std::endl << std::endl;
}

void RowColRange::RunColRangeDemo()
{
    cv::Mat mat = (cv::Mat_<double>(3, 3) << 0, 1, 2, 3, 4, 5, 6, 7, 8);
    std::cout << "matrix: " << std::endl
              << mat << std::endl << std::endl;

    cv::Mat col = mat.colRange(0, 2).clone();   // 包括左边界，但不包括右边界
    std::cout << "Col range:" << std::endl
              << col << std::endl << std::endl;
}


} // namespace image_basic
} // namespace opencv
} // namespace xslam
