.. highlight:: c++

.. default-domain:: cpp

====================
Geometry
====================

1 Fundamentals Matrix
=====================

Opencv C++ API:

.. code-block:: c++

    CV_EXPORTS_W Mat findFundamentalMat( InputArray points1, InputArray points2,
                                         int method = FM_RANSAC,
                                         double param1 = 3., double param2 = 0.99,
                                         OutputArray mask = noArray() );
.. NOTE::

    * points1：第一张图像的N个点；
    * points2: 第二张图像的点；
    * param1: 该参数用于RANSAC算法（随机采样过程一致性），它是从点到对极线的最大距离（以像素为单位），超过该距离，该点被视为异常值，并且不用于计算最终的基础矩阵。 可以将其设置为1-3，具体取决于点定位的精度，图像分辨率和图像噪声。
    * param2: 该参数仅仅在RANSAC算法以及LMedS算法中， 它指定了估计矩阵正确的期望置信度（概率）。

**计算基础矩阵的方法**

    * 7点法

    * 8点法 矩阵E中有九个参数，根据尺度不变性，可以通过八对点估计基础矩阵，也就是所谓的八点法

    * RANSAC算法

    * LMedS算法

.. code-block:: c++

    enum 
    { 
        FM_7POINT = 1, //!< 7-point algorithm
        FM_8POINT = 2, //!< 8-point algorithm
        FM_LMEDS  = 4, //!< least-median algorithm
        FM_RANSAC = 8  //!< RANSAC algorithm
    };

demo调用

.. code-block:: c++

    TEST(PoseEstimation2D2D, pose_estimation_2d)
    {
        std::string image1 = GetOpenCVDatasetDirectory() + "/0003_orb_feature_detector.png";
        std::string image2 = GetOpenCVDatasetDirectory() + "/0004_orb_feature_detector.png";

        // OpenCV
        PoseEstimation2D2D demo;
        demo.RunDemo(image1, image2);
    }

函数使用：

.. code-block:: c++

    /**
     * 本程序演示了如何使用2D-2D的特征匹配估计相机运动
     */
    class PoseEstimation2D2D
    {
    public:
        void RunDemo(const std::string& iamge_1, const std::string& iamge_2);

    private:
        void FindFeatureMatches(
                const cv::Mat &img_1, const cv::Mat &img_2,
                std::vector<cv::KeyPoint> &keypoints_1,
                std::vector<cv::KeyPoint> &keypoints_2,
                std::vector<cv::DMatch> &matches);

        void PoseEstimation_2D2D(
                std::vector<cv::KeyPoint> keypoints_1,
                std::vector<cv::KeyPoint> keypoints_2,
                std::vector<cv::DMatch> matches,
                cv::Mat &R, cv::Mat &t);

        // 像素坐标转相机归一化坐标
        cv::Point2d Pixel2Cam(const cv::Point2d &p, const cv::Mat &K);
    };


    void PoseEstimation2D2D::RunDemo(const std::string& iamge_1, const std::string& iamge_2)
    {
        //-- 读取图像
        cv::Mat img_1 = cv::imread(iamge_1);
        cv::Mat img_2 = cv::imread(iamge_2);
        assert(img_1.data && img_2.data && "Can not load images!");

        std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
        std::vector<cv::DMatch> matches;
        FindFeatureMatches(img_1, img_2, keypoints_1, keypoints_2, matches);
        std::cout << "一共找到了" << matches.size() << "组匹配点" << std::endl;

        //-- 估计两张图像间运动
        cv:: Mat R, t;
        PoseEstimation_2D2D(keypoints_1, keypoints_2, matches, R, t);

        //-- 验证E=t^R*scale
        cv::Mat t_x =
                (cv::Mat_<double>(3, 3) << 0, -t.at<double>(
                        2, 0), t.at<double>(1, 0),
                        t.at<double>(2, 0), 0, -t.at<double>(0, 0),
                        -t.at<double>(1, 0), t.at<double>(0, 0), 0);

        std::cout << "t^R=" << std::endl << t_x * R << std::endl;

        //-- 验证对极约束
        cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
        for (cv::DMatch m: matches)
        {
            cv::Point2d pt1 = Pixel2Cam(keypoints_1[m.queryIdx].pt, K);
            cv::Mat y1 = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
            cv::Point2d pt2 = Pixel2Cam(keypoints_2[m.trainIdx].pt, K);
            cv::Mat y2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
            cv::Mat d = y2.t() * t_x * R * y1;
            std::cout << "epipolar constraint = " << d << std::endl;
        }
    }

    void PoseEstimation2D2D::FindFeatureMatches(
            const cv::Mat &img_1, const cv::Mat &img_2,
            std::vector<cv::KeyPoint> &keypoints_1,
            std::vector<cv::KeyPoint> &keypoints_2,
            std::vector<cv::DMatch> &matches)
    {
        //-- 初始化
        cv::Mat descriptors_1, descriptors_2;
        // used in OpenCV3
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

        // use this if you are in OpenCV2
        // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
        // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

        //-- 第一步:检测 Oriented FAST 角点位置
        detector->detect(img_1, keypoints_1);
        detector->detect(img_2, keypoints_2);

        //-- 第二步:根据角点位置计算 BRIEF 描述子
        descriptor->compute(img_1, keypoints_1, descriptors_1);
        descriptor->compute(img_2, keypoints_2, descriptors_2);

        //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
        std::vector<cv::DMatch> match;
        //BFMatcher matcher ( NORM_HAMMING );
        matcher->match(descriptors_1, descriptors_2, match);

        //-- 第四步:匹配点对筛选
        double min_dist = 10000, max_dist = 0;

        //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
        for (int i = 0; i < descriptors_1.rows; i++) {
            double dist = match[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }

        printf("-- Max dist : %f \n", max_dist);
        printf("-- Min dist : %f \n", min_dist);

        //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
        for (int i = 0; i < descriptors_1.rows; i++) {
            if (match[i].distance <= std::max(2 * min_dist, 30.0)) {
                matches.push_back(match[i]);
            }
        }
    }

    void PoseEstimation2D2D::PoseEstimation_2D2D(
            std::vector<cv::KeyPoint> keypoints_1,
            std::vector<cv::KeyPoint> keypoints_2,
            std::vector<cv::DMatch> matches,
            cv::Mat &R, cv::Mat &t)
    {
        // 相机内参,TUM Freiburg2
        cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

        //-- 把匹配点转换为vector<Point2f>的形式
        std::vector<cv::Point2f> points1;
        std::vector<cv::Point2f> points2;

        for (int i = 0; i < (int) matches.size(); i++) {
            points1.push_back(keypoints_1[matches[i].queryIdx].pt);
            points2.push_back(keypoints_2[matches[i].trainIdx].pt);
        }

        //-- 计算基础矩阵
        cv::Mat fundamental_matrix;
        fundamental_matrix = cv::findFundamentalMat(points1, points2, cv::FM_8POINT);
        std::cout << "fundamental_matrix is " << std::endl << fundamental_matrix << std::endl;

        //-- 计算本质矩阵
        cv::Point2d principal_point(325.1, 249.7);  //相机光心, TUM dataset标定值
        double focal_length = 521;      //相机焦距, TUM dataset标定值
        cv::Mat essential_matrix;
        essential_matrix = cv::findEssentialMat(points1, points2, focal_length, principal_point);
        std::cout << "essential_matrix is " << std::endl << essential_matrix << std::endl;

        //-- 计算单应矩阵
        //-- 但是本例中场景不是平面，单应矩阵意义不大
        cv::Mat homography_matrix;
        homography_matrix = cv::findHomography(points1, points2, cv::RANSAC, 3);
        std::cout << "homography_matrix is " << std::endl << homography_matrix << std::endl;

        //-- 从本质矩阵中恢复旋转和平移信息.
        // 此函数仅在Opencv3中提供
        cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
        std::cout << "R is " << std::endl << R << std::endl;
        std::cout << "t is " << std::endl << t << std::endl;
    }

    cv::Point2d PoseEstimation2D2D::Pixel2Cam(const cv::Point2d &p, const cv::Mat &K)
    {
        return cv::Point2d
        {
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
        };
    }


运行结果

.. code-block:: bash

    [bin] ./xslam.opencv.geometry_transform.pose_estimation_2d2d_test

    -- Max dist : 94.000000 
    -- Min dist : 4.000000 
    一共找到了79组匹配点
    fundamental_matrix is 
    [4.54443750398184e-06, 0.0001333855576992603, -0.01798499246479044;
    -0.0001275657012964255, 2.266794804645652e-05, -0.01416678429206633;
    0.01814994639971766, 0.004146055870980492, 1]
    essential_matrix is 
    [-0.008455114492964278, 0.05451570701059781, 0.1546375809484052;
    -0.008287154708445212, 0.03351311565984172, -0.6896472136971504;
    -0.1153993974485718, 0.6945899967012867, 0.02159624094256633]
    homography_matrix is 
    [0.9261214237658335, -0.1445322040802305, 33.26921164265664;
    0.04535424230636757, 0.9386696658342905, 8.570980713233848;
    -1.006198269424755e-05, -3.008140685985328e-05, 1]
    R is 
    [0.9956584940813579, -0.05615340406690447, 0.07423582945816433;
    0.05268846331440004, 0.9974645001566195, 0.04783823534446425;
    -0.07673388428334535, -0.0437191735855581, 0.9960926386957119]
    t is 
    [-0.9726703113454949;
    -0.2153829834753195;
    0.08673313009645391]
    t^R=
    [0.01195733758736675, -0.07709685221674556, -0.2186905642298021;
    0.01171980658216709, -0.04739470268352609, 0.9753084428633267;
    0.1631993929614534, -0.9822985936236425, -0.03054169683725466]
    epipolar constraint = [-0.0005617285518606241]
    epipolar constraint = [0.002891683190146016]
    epipolar constraint = [-0.0001941259398173245]
    epipolar constraint = [0.003462947761727536]
    epipolar constraint = [8.120001470268701e-06]
    ...
    epipolar constraint = [0.005653889777384447]
    epipolar constraint = [0.0008830143247820065]
    epipolar constraint = [-0.001103292290051336]
    epipolar constraint = [-0.003982708195313309]
    epipolar constraint = [-0.0053874915375101]

参考源码：

.. NOTE::

    * pose_estimation_2d2d_test.cpp
    * pose_estimation_2d2d.cpp
    * pose_estimation_2d2d.h

2 Essential Matrix
==================

.. code-block:: c++

   
demo调用

.. code-block:: c++



函数使用：

.. code-block:: c++



3 PnP
=====
