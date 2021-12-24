#ifndef XSLAM_OPENCV_PAINT_SHAPE_H
#define XSLAM_OPENCV_PAINT_SHAPE_H

#include "glog/logging.h"
#include "opencv2/opencv.hpp"

namespace xslam {
namespace opencv {

class PaintShape
{
public:
    void RunDemo();

private:
    // 要绘制一条线，您需要传递线的开始和结束坐标。我们将创建一个黑色图像，并从左上角到右下角在其上绘制一条蓝线
    void PaintLine();

    // 要绘制一个圆，需要其中心坐标和半径。我们将在上面绘制的矩形内绘制一个圆
    void PaintCircle();

    // 要绘制矩形，您需要矩形的左上角和右下角。这次，我们将在图像的右上角绘制一个绿色矩形
    void PaintRectangle();

    // 要绘制椭圆，我们需要传递几个参数。一个参数是中心位置（x，y）。
    // 下一个参数是轴长度（长轴长度，短轴长度）。angle是椭圆沿逆时针方向旋转的角度。
    // startAngle和endAngle表示从主轴沿顺时针方向测量的椭圆弧的开始和结束。
    // 即给出0和360给出完整的椭圆。
    void PaintEllipse();

    // 要绘制多边形，首先需要顶点的坐标。将这些点组成形状为ROWSx1x2的数组，其中ROWS是顶点数，
    // 并且其类型应为int32。在这里，我们绘制了一个带有四个顶点的黄色小多边形。
    void PaintPolylines();

    // 要将文本放入图像中，需要指定以下内容。 
    // - 您要写入的文字数据 - 您要放置它的位置坐标（即数据开始的左下角）。 
    // - 字体类型（检查**cv.putText**文档以获取受支持的字体） - 字体比例（指定字体大小） - 常规的内容，例如颜色，厚度，线条类型等。
    // 为了获得更好的外观，建议使用lineType = cv.LINE_AA。
    void PaintText(); 
};

} // namespace opencv
} // namespace xslam

#endif // XSLAM_OPENCV_PAINT_SHAPE_H