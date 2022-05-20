#include "xslam/opencv/image_processing/matchTemplate.h"

namespace xslam {
namespace opencv {
namespace image_processing {

/**
 * @brief cv::matchTemplate
 * 
 * 参数1:src用于搜索的输入图像, 8U 或 32F, 大小 W-H
 * 参数2:用于匹配的模板，和src类型相同， 大小 w-h
 * 参数3:匹配结果图像, 类型 32F, 大小 (W-w+1)-(H-h+1)
 * 参数4:用于比较的方法(有六种)
        cv::TM_SQDIFF=0 该方法使用平方差进行匹配，因此最佳的匹配结果在结果为0处，值越大匹配结果越差
        cv::TM_SQDIFF_NORMED=1：该方法使用归一化的平方差进行匹配，最佳匹配也在结果为0处
        cv::TM_CCORR=2：相关性匹配方法，该方法使用源图像与模板图像的卷积结果进行匹配，因此，最佳
            匹配位置在值最大处，值越小匹配结果越差  【个人测试：匹配性很差】
        cv::TM_CCORR_NORMED=3：归一化的相关性匹配方法，与相关性匹配方法类似，最佳匹配位置也是在值最大处
        cv::TM_CCOEFF=4：相关性系数匹配方法，该方法使用源图像与其均值的差、模板与其均值的差二者之间的相
            关性进行匹配，最佳匹配结果在值等于1处，最差匹配结果在值等于-1处，值等于0直接表示二者不相关
        cv::TM_CCOEFF_NORMED=5：归一化的相关性系数匹配方法，正值表示匹配的结果较好，负值则表示匹配的效
            果较差，也是值越大，匹配效果也好
 */

void MatchTemplate::RunDemo(const std::string& filename, const std::string& template_file)
{
    // 加载源图像和模板图像
    cv::Mat image = cv::imread(filename);
    if (image.data == nullptr) {
        std::cout << "Load image error." << std::endl;
        exit(-1);
    }
	
    cv::Mat template_image = cv::imread(template_file);
    if (template_image.data == nullptr) {
        std::cout << "Load template_image error." << std::endl;
        exit(-1);
    }

    cv::Mat ftmp;
    cv::matchTemplate(image, template_image, ftmp, 5); //模板匹配
    std::cerr << cv::TM_CCOEFF_NORMED << std::endl;

    cv::normalize(ftmp, ftmp, 1, 0, cv::NORM_MINMAX); // 可以不归一化
    double minVal; double maxVal;
    cv::Point minLoc; 
    cv::Point maxLoc;
    cv::minMaxLoc(ftmp, &minVal, &maxVal, &minLoc, &maxLoc); // 找到最佳匹配点

    // 从匹配结果图像中找出最佳匹配点
    cv::rectangle(image, cv::Rect(maxLoc.x, maxLoc.y, template_image.cols, 
        template_image.rows), cv::Scalar(0, 0, 255), 2, 8); // 画出匹配到的矩形框

    cv::imshow("image", image);

    while (true) 
    {
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
