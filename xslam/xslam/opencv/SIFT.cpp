#include "xslam/opencv/SIFT.h"

#include <vector>

namespace xslam {
namespace opencv {
    
void SIFTFeature::RunDemo(const std::string& filename)
{
    cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    cv::Mat color_img = cv::imread(filename);

    if (image.data == nullptr || color_img.data == nullptr) {
        std::cout << "Load image error." << filename << std::endl;
        exit(-1);
    }


    cv::Mat float_img;
    image.convertTo(float_img,CV_32F);

    int rows = image.rows;
    int cols = image.cols;
    vl_sift_ =  vl_sift_new(cols, rows, 4, 3, 0);
    vl_sift_set_peak_thresh(vl_sift_, 0.04);
    vl_sift_set_edge_thresh(vl_sift_, 10);

    vl_sift_pix *data = (vl_sift_pix*)(float_img.data);


    std::vector<VlSiftKeypoint> vlfeat_keypoints;
    std::vector<cv::KeyPoint>   opencv_keypoints;

    std::vector<float*> descriptors;

    ExtractFeature(vl_sift_,data, vlfeat_keypoints, descriptors);
    ConvertToOpencvKeypoint(vlfeat_keypoints, opencv_keypoints);

    drawKeypoints(image, opencv_keypoints, image);


    imshow("SIFT Feature", image);
    cv::waitKey(0);
    cv::destroyAllWindows();
    vl_sift_delete(vl_sift_);
}

void SIFTFeature::ExtractFeature(VlSiftFilt* sift_ptr, vl_sift_pix* data, 
    std::vector<VlSiftKeypoint>& keypoints, std::vector<float*>& descriptors)
{
    // Detect keypoint and compute descriptor in each octave
    if(vl_sift_process_first_octave(vl_sift_, data) != VL_ERR_EOF)
    {
        while(true)
        {
            vl_sift_detect(vl_sift_);

            VlSiftKeypoint* pKpts = vl_sift_->keys;
            for(int i = 0; i < vl_sift_->nkeys; i ++) 
            {

                double angles[4];
                // 计算特征点的方向，包括主方向和辅方向，最多4个
                int angleCount = vl_sift_calc_keypoint_orientations(vl_sift_, angles, pKpts);

                // 对于方向多于一个的特征点，每个方向分别计算特征描述符
                // 并且将特征点复制多个
                for(int i = 0 ; i < angleCount; i ++)
                {
                    float *des = new float[128];
                    vl_sift_calc_keypoint_descriptor(vl_sift_, des, pKpts, angles[0]);
                    descriptors.push_back(des);
                    keypoints.push_back(*pKpts);
                }

                pKpts ++;
            }    
            // Process next octave
            if(vl_sift_process_next_octave(vl_sift_) == VL_ERR_EOF) 
            {
                break ;
            }
        }
    }
}

void SIFTFeature::ConvertToOpencvKeypoint(
    std::vector<VlSiftKeypoint>& vlfeat_keypoints,
    std::vector<cv::KeyPoint>& opencv_keypoints)
{
    for (auto keypoint : vlfeat_keypoints) {

        opencv_keypoints.push_back({keypoint.x, keypoint.y, keypoint.sigma});
    }
}


} // namespace opencv
} // namespace xslam
