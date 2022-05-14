#include "xslam/opencv/image_processing/geometry_transform.h"

namespace xslam {
namespace opencv {
namespace image_processing {

void GeometryTransform::RunDemo(const std::string& filename)
{
    // 0. 读取图像
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }
    // 原图
    cv::imshow("image", image); // 展示源图像

    // resize
    Resize(image);

    // AffineTransform
    AffineTransform(image);

    // WarpPerspective
    WarpPerspective(image);

    cv::waitKey(0);
    cv::destroyAllWindows();
}

void GeometryTransform::Resize(const cv::Mat& image)
{
    cv::Mat dst;
    cv::resize(image, dst, cv::Size(200, 200), 0, 0, cv::INTER_CUBIC);
    cv::imshow("resize", dst);
    cv::waitKey(0);
}

void GeometryTransform::AffineTransform(const cv::Mat& image)
{
    cv::Point2f srcTri[3];
    cv::Point2f dstTri[3];

    cv::Mat rot_mat( 2, 3, CV_32FC1 );
    cv::Mat warp_mat( 2, 3, CV_32FC1 );
    cv::Mat warp_dst, warp_rotate_dst;

    // 设置目标图像的大小和类型与源图像一致
    warp_dst = cv::Mat::zeros( image.rows, image.cols, image.type() );

    /// 设置源图像和目标图像上的三组点以计算仿射变换
    srcTri[0] = cv::Point2f( 0,0 );
    srcTri[1] = cv::Point2f( image.cols - 1, 0 );
    srcTri[2] = cv::Point2f( 0, image.rows - 1 );

    dstTri[0] = cv::Point2f( image.cols*0.0, image.rows*0.33 );
    dstTri[1] = cv::Point2f( image.cols*0.85, image.rows*0.25 );
    dstTri[2] = cv::Point2f( image.cols*0.15, image.rows*0.7 );

    /// 求得仿射变换
    warp_mat = cv::getAffineTransform( srcTri, dstTri );

    /// 对源图像应用上面求得的仿射变换
    cv::warpAffine( image, warp_dst, warp_mat, warp_dst.size() );

    /** 对图像扭曲后再旋转 */

    // 计算绕图像中点顺时针旋转50度缩放因子为0.6的旋转矩阵
    cv::Point center = cv::Point( warp_dst.cols/2, warp_dst.rows/2 );
    double angle = -50.0;
    double scale = 0.6;

    // 通过上面的旋转细节信息求得旋转矩阵
    rot_mat = cv::getRotationMatrix2D( center, angle, scale );

    // 旋转已扭曲图像
    cv::warpAffine( warp_dst, warp_rotate_dst, rot_mat, warp_dst.size() );

    // 显示结果
    cv::namedWindow("source", cv::WINDOW_AUTOSIZE );
    cv::imshow("source", image );

    cv::namedWindow("warp", cv::WINDOW_AUTOSIZE );
    cv::imshow("warp", warp_dst );

    cv::namedWindow("warp_rotate", cv::WINDOW_AUTOSIZE );
    cv::imshow("warp_rotate", warp_rotate_dst );

    // 等待用户按任意按键退出程序
    cv::waitKey(0);
}

void GeometryTransform::WarpPerspective(const cv::Mat& image)
{
    cv::Point2f srcTri[4];
	cv::Point2f dstTri[4];
 
	cv::Mat warpPerspective_mat( 3, 3, CV_32FC1 );
	cv::Mat warpPerspective_dst;
 
	// Set the dst image the same type and size as src
	warpPerspective_dst = cv::Mat::zeros( image.rows, image.cols, image.type() );
 
	/// 设置三组点，求出变换矩阵
	srcTri[0] = cv::Point2f( 0,0 );
	srcTri[1] = cv::Point2f( image.cols - 1,0 );
	srcTri[2] = cv::Point2f( 0, image.rows - 1);
	srcTri[3] = cv::Point2f(image.cols - 1, image.rows - 1);
 
	dstTri[0] = cv::Point2f( 0, image.rows * 0.13 );
	dstTri[1] = cv::Point2f( image.cols * 0.9, 0 );
	dstTri[2] = cv::Point2f( image.cols * 0.2, image.rows * 0.7 );
	dstTri[3] = cv::Point2f( image.cols * 0.8, image.rows );
 
	//计算3个二维点对之间的仿射变换矩阵（2行x3列）
	warpPerspective_mat = cv::getPerspectiveTransform( srcTri, dstTri );
 
	//应用仿射变换，可以恢复出原图
	warpPerspective( image, warpPerspective_dst, warpPerspective_mat, image.size() );
 
	//显示结果
	cv::namedWindow("source", cv::WINDOW_AUTOSIZE );
	cv::imshow("source", image);
 
	cv::namedWindow("warpPerspective", cv::WINDOW_AUTOSIZE );
	cv::imshow("warpPerspective", warpPerspective_dst );
    cv::waitKey(0);
}


} // namespace image_processing
} // namespace opencv
} // namespace xslam
