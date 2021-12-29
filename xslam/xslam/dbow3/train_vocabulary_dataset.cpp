#include "xslam/dbow3/train_vocabulary_dataset.h"
#include "DBoW3/DBoW3.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <vector>

#include "glog/logging.h"

namespace xslam {
namespace dbow3 {

void VocabularyTrain::RunDemo(const std::string& path)
{
    // read the image 
    LOG(INFO) << "Reading images...";
    std::vector<cv::Mat> images; 

    std::cout << "path = " << path << std::endl;
    for (int i = 0; i < 10; i++ )
    {
        std::string image = path + std::to_string(i+1)+".png";
        images.push_back(cv::imread(image) );
    }

    // detect ORB features
    LOG(INFO) << "detecting ORB features ... ";
    cv::Ptr<cv::Feature2D > detector = cv::ORB::create();

    std::vector<cv::Mat> descriptors;
    for (cv::Mat& image:images )
    {
        std::vector<cv::KeyPoint> keypoints; 
        cv::Mat descriptor;
        detector->detectAndCompute(image, cv::Mat(), keypoints, descriptor);
        descriptors.push_back( descriptor );
    }
    
    // create vocabulary 
    LOG(INFO)  <<"creating vocabulary ... ";;
    DBoW3::Vocabulary vocab;
    vocab.create( descriptors );
    LOG(INFO) << "vocabulary info: "<< vocab;;
    vocab.save("vocabulary.yml.gz");
    LOG(INFO) << "done";;
}

} // namespace dbow3
} // namespace xslam