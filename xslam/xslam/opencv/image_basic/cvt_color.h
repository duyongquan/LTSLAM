#ifndef XSLAM_OPENCV_IMAGE_BASIC_CVT_COLOR_H
#define XSLAM_OPENCV_IMAGE_BASIC_CVT_COLOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <boost/dynamic_bitset.hpp>

namespace xslam {
namespace opencv {
namespace image_basic {

// 我们生活中大多数看到的彩色图片都是RGB类型，但是在进行图像处理时，需要用到灰度图、二值图、HSV、HSI等颜色制式，
// opencv提供了cvtColor()函数来实现这些功能。首先看一下cvtColor函数定义：
// void cvtColor(InputArray src, OutputArray dst, int code, int dstCn=0 );

// 参数解释：
//   InputArray src: 输入图像即要进行颜色空间变换的原图像，可以是Mat类
//   OutputArray dst: 输出图像即进行颜色空间变换后存储图像，也可以Mat类
//   int code: 转换的代码或标识，即在此确定将什么制式的图片转换成什么制式的图片，
//   int dstCn = 0: 目标图像通道数，如果取值为0，则由src和code决定

// 函数的作用是将一个图像从一个颜色空间转换到另一个颜色空间，但是从RGB向其他类型转换时，必须明确指出图像的颜色通道，
// 前面我们也提到过，在opencv中，其默认的颜色制式排列是BGR而非RGB。所以对于24位颜色图像来说，前8-bit是蓝色，中间8-bit是绿色，最后8-bit是红色。
// 常见的R,G,B通道的取值范围为：
//   0-255   :  CV_8U类型图片
//   0-65535 :  CV_16U类型图片
//   0-1     :  CV_32F类型图片
// 对于线性变换来说，这些取值范围是无关紧要的。但是对于非线性转换，输入的RGB图像必须归一化到其对应的取值范围来或得最终正确的转换结果，
// 例如从RGB->L*u*v转换。如果从一个8-bit类型图像不经过任何缩放（scaling）直接转换为32-bit浮点型图像，
// 函数将会以0-255的取值范围来取代0-1的取值范围，所以在使用cvtColor函数之前需要对图像进行缩放如下：

class CvtColor
{
public:
    void RunDemo(const std::string& filename);
};

} // namespace image_basic
} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_IMAGE_BASIC_CVT_COLOR_H