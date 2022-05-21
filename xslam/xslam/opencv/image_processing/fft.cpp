#include "xslam/opencv/image_processing/fft.h"

#include <vector>

namespace xslam {
namespace opencv {
namespace image_processing {

void FFT::RunDemo(const std::string& filename)
{
    // 0. 读取图像
    cv::Mat image = cv::imread(filename, 0);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }

	cv::imshow("原图", image);
    // 将输入图像扩展到最佳尺寸，边界用0填充
    // 离散傅里叶变换的运行速度与图像的大小有很大的关系，当图像的尺寸使2，3，5的整数倍时，计算速度最快
    // 为了达到快速计算的目的，经常通过添加新的边缘像素的方法获取最佳图像尺寸
    // 函数getOptimalDFTSize()用于返回最佳尺寸，copyMakeBorder()用于填充边缘像素
    int m = cv::getOptimalDFTSize(image.rows);
    int n = cv::getOptimalDFTSize(image.cols);
 
    cv::Mat padded;
    cv::copyMakeBorder(image, padded, 0, m - image.rows, 0, n - image.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
 
    // 为傅立叶变换的结果分配存储空间
    // 将plannes数组组合成一个多通道的数组，两个同搭配，分别保存实部和虚部
    // 傅里叶变换的结果使复数，这就是说对于每个图像原像素值，会有两个图像值
    // 此外，频域值范围远远超过图象值范围，因此至少将频域储存在float中
    // 所以我们将输入图像转换成浮点型，并且多加一个额外通道来存储复数部分
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
 
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);

    // 进行离散傅立叶变换
    cv::dft(complexI, complexI);
 
    // 将复数转化为幅值，保存在planes[0]
    cv::split(complexI, planes);   // 将多通道分为几个单通道
    cv::magnitude(planes[0], planes[1], planes[0]);
    cv::Mat magnitudeImage = planes[0];
 
    // 傅里叶变换的幅值达到不适合在屏幕上显示，因此我们用对数尺度来替换线性尺度
    // 进行对数尺度logarithmic scale缩放
    magnitudeImage += cv::Scalar::all(1);     // 所有的像素都加1
    log(magnitudeImage, magnitudeImage);      // 求自然对数
 
    // 剪切和重分布幅度图像限
    // 如果有奇数行或奇数列，进行频谱裁剪
    magnitudeImage = magnitudeImage(cv::Rect(0, 0, magnitudeImage.cols & -2, magnitudeImage.rows & -2));
 
    // ---- -------- 下面的是为了显示结果 ---------------
    // 一分为四，左上与右下交换，右上与左下交换
    // 重新排列傅里叶图像中的象限，使原点位于图像中心
    int cx = magnitudeImage.cols / 2;
    int cy = magnitudeImage.rows / 2;
    cv::Mat q0(magnitudeImage, cv::Rect(0, 0, cx, cy));   // ROI区域的左上
    cv::Mat q1(magnitudeImage, cv::Rect(cx, 0, cx, cy));  // ROI区域的右上
    cv::Mat q2(magnitudeImage, cv::Rect(0, cy, cx, cy));  // ROI区域的左下
    cv::Mat q3(magnitudeImage, cv::Rect(cx, cy, cx, cy)); // ROI区域的右下
 
    // 交换象限（左上与右下进行交换）
    cv::Mat tmp;
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    // 交换象限（右上与左下进行交换）
    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);
 
    // 归一化
    cv::normalize(magnitudeImage, magnitudeImage, 0, 1, cv::NORM_MINMAX);
    // 显示效果图
    cv::imshow("频谱幅值", magnitudeImage);
    while (true) {
        if (27 == cv::waitKey()) {
            break;
        } 
        sleep(1);
    }
    cv::destroyAllWindows();
}

} // namespace image_processing
} // namespace opencv
} // namespace xslam
