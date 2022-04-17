#include "xslam/g2o/g2o_ba.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace xslam {
namespace g2o {


void SimpleBA::RunDemo(const std::string& image1, const std::string& image2)
{
    // 读取图像
    cv::Mat img1 = cv::imread(image1);
    cv::Mat img2 = cv::imread(image2);

    // 找到对应点
    std::vector<cv::Point2f> pts1, pts2;
    if (FindCorrespondingPoints( img1, img2, pts1, pts2 ) == false)
    {
        std::cout << "匹配点不够！" << std::endl;
        return;
    }

    std::cout << "找到了" << pts1.size() << "组对应特征点。"<< std::endl;


    // 构造g2o中的图
    // 先构造求解器
    ::g2o::SparseOptimizer optimizer;

    // 使用Cholmod中的线性方程求解器
    auto linearSolver = ::g2o::make_unique<::g2o::LinearSolverCholmod<::g2o::BlockSolver_6_3::PoseMatrixType>>();

    // 6*3 的参数
    // L-M 下降
    ::g2o::OptimizationAlgorithmLevenberg* algorithm = new ::g2o::OptimizationAlgorithmLevenberg(
        ::g2o::make_unique<::g2o::BlockSolver_6_3>(std::move(linearSolver)));

    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(false );

    // 添加节点
    // 两个位姿节点
    for (int i = 0; i < 2; i++)
    {
        ::g2o::VertexSE3Expmap* v = new ::g2o::VertexSE3Expmap();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true ); // 第一个点固定为零
        // 预设值为单位Pose，因为我们不知道任何信息
        v->setEstimate(::g2o::SE3Quat() );
        optimizer.addVertex( v );
    }

    // 很多个特征点的节点
    // 以第一帧为准
    for ( size_t i = 0; i < pts1.size(); i++)
    {
        ::g2o::VertexSBAPointXYZ* v = new ::g2o::VertexSBAPointXYZ();
        v->setId( 2 + i );

        // 由于深度不知道，只能把深度设置为1了
        double z = 1;
        double x = ( pts1[i].x - cx ) * z / fx;
        double y = ( pts1[i].y - cy ) * z / fy;
        v->setMarginalized(true);
        v->setEstimate(Eigen::Vector3d(x,y,z) );
        optimizer.addVertex( v );
    }


    // 准备相机参数
    ::g2o::CameraParameters* camera = new ::g2o::CameraParameters(fx, Eigen::Vector2d(cx, cy), 0);
    camera->setId(0);
    optimizer.addParameter( camera );

    // 准备边
    // 第一帧
    std::vector<::g2o::EdgeProjectXYZ2UV*> edges;
    for ( size_t i = 0; i < pts1.size(); i++ )
    {
        ::g2o::EdgeProjectXYZ2UV* edge = new ::g2o::EdgeProjectXYZ2UV();
        edge->setVertex(0, dynamic_cast<::g2o::VertexSBAPointXYZ*>(optimizer.vertex(i+2)));
        edge->setVertex(1, dynamic_cast<::g2o::VertexSE3Expmap*>(optimizer.vertex(0)));
        edge->setMeasurement(Eigen::Vector2d(pts1[i].x, pts1[i].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setParameterId(0, 0);

        // 核函数
        edge->setRobustKernel(new ::g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }

    // 第二帧
    for ( size_t i = 0; i < pts2.size(); i++)
    {
        ::g2o::EdgeProjectXYZ2UV*  edge = new ::g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<::g2o::VertexSBAPointXYZ*>(optimizer.vertex(i+2)));
        edge->setVertex( 1, dynamic_cast<::g2o::VertexSE3Expmap*>(optimizer.vertex(1)));
        edge->setMeasurement( Eigen::Vector2d(pts2[i].x, pts2[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0,0);

        // 核函数
        edge->setRobustKernel(new ::g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }

    std::cout<< "开始优化 "<< std::endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    std::cout<< "优化完毕" << std::endl;

    //我们比较关心两帧之间的变换矩阵
    ::g2o::VertexSE3Expmap* v = dynamic_cast<::g2o::VertexSE3Expmap*>(optimizer.vertex(1));
    Eigen::Isometry3d pose = v->estimate();
    std::cout << "Pose=" << std::endl << pose.matrix() << std::endl;

    // 以及所有特征点的位置
    for (size_t i = 0; i < pts1.size(); i++)
    {
        ::g2o::VertexSBAPointXYZ* v = dynamic_cast<::g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
        std::cout << "vertex id " << i + 2 << ", pos = ";
        Eigen::Vector3d pos = v->estimate();
        std::cout << pos(0) << "," << pos(1) << "," << pos(2) << std::endl;
    }

    // 估计inlier的个数
    int inliers = 0;
    for ( auto e : edges ) {
        e->computeError();
        // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
        if ( e->chi2() > 1 ) {
            std::cout<<"error = " << e->chi2() << std::endl;
        }
        else {
            inliers++;
        }
    }

    std::cout << "inliers in total points: "<< inliers << "/" << pts1.size() + pts2.size() << std::endl;
    optimizer.save("ba.g2o");
}
  


int SimpleBA::FindCorrespondingPoints(
    const cv::Mat& img1, const cv::Mat& img2, 
    std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2)
{
    //-- 初始化
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img1, keypoints_1);
    detector->detect(img2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img1, keypoints_1, descriptors_1);
    descriptor->compute(img2, keypoints_2, descriptors_2);

    // cv::Mat outimg1;
    // drawKeypoints(img_1, keypoints_1, outimg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    // imshow("ORB features", outimg1);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);

    //-- 第四步:匹配点对筛选
    // 计算最小距离和最大距离
    auto min_max = minmax_element(matches.begin(), matches.end(),
            [](const cv::DMatch &m1, const cv::DMatch &m2) {
        return m1.distance < m2.distance;
    });

    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (matches[i].distance <= std::max(2 * min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
        }
    }

    //-- 第五步:绘制匹配结果
    cv::Mat img_match;
    cv::Mat img_goodmatch;
    drawMatches(img1, keypoints_1, img1, keypoints_2, matches, img_match);
    drawMatches(img2, keypoints_1, img2, keypoints_2, good_matches, img_goodmatch);
    imshow("all matches", img_match);
    imshow("good matches", img_goodmatch);
    cv::waitKey(0);
    
    // 匹配点太少
    if (good_matches.size() <= 20) {
        return false;
    }
        
    for ( auto m : good_matches )
    {
        points1.push_back(keypoints_1[m.queryIdx].pt);
        points2.push_back(keypoints_2[m.trainIdx].pt);
    }
    
    return true;
}

} // namespace g2o
} // namespace xslam