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


运行结果

.. code-block:: bash

    [bin] ./xslam.opencv.geometry_transform.pose_estimation_2d2d_test

3 PnP
=====

Opencv C++ API:

.. code-block:: c++

   void solvePnP(
        InputArray objectPoints, 
        InputArray imagePoints, 
        InputArray cameraMatrix, InputArray distCoeffs, 
        OutputArray rvec, OutputArray tvec, 
        bool useExtrinsicGuess=false, 
        int flags = CV_ITERATIVE)


.. NOTE::

    * objectPoints - 世界坐标系下的控制点的坐标，vector的数据类型在这里可以使用
    * imagePoints - 在图像坐标系下对应的控制点的坐标。vector在这里可以使用
    * cameraMatrix - 相机的内参矩阵
    * distCoeffs - 相机的畸变系数
    * rvec - 输出的旋转向量。使坐标点从世界坐标系旋转到相机坐标系
    * tvec - 输出的平移向量。使坐标点从世界坐标系平移到相机坐标系
    * flags - 默认使用CV_ITERATIV迭代法

**solvePnP里有三种解法**

* P3P、 EPnP、迭代法(默认);
* opencv2里对应的参数分别为: CV_P3P、CV_EPNP、CV_ITERATIVE(opencv3里多了DLS和UPnP解法)

demo调用

.. code-block:: c++

    TEST(PoseEstimation3D2D, pose_estimation_2d)
    {
        std::string image1 = GetOpenCVDatasetDirectory() + "/0024_1.png";
        std::string depth1 = GetOpenCVDatasetDirectory() + "/0024_1_depth.png";
        std::string image2 = GetOpenCVDatasetDirectory() + "/0024_2.png";
        std::string depth2 = GetOpenCVDatasetDirectory() + "/0024_2_depth.png";

        // OpenCV
        PoseEstimation3D2D demo;
        demo.RunDemo(image1, depth1, image2, depth2);
    }

函数使用：

.. code-block:: c++

    // vertex and edges used in g2o ba
    class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        virtual void setToOriginImpl() override 
        {
            _estimate = Sophus::SE3d();
        }

        // left multiplication on SE3
        virtual void oplusImpl(const double *update) override 
        {
            Eigen::Matrix<double, 6, 1> update_eigen;
            update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
            _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
        }

        virtual bool read(std::istream &in) override {}
        virtual bool write(std::ostream &out) const override {}
    };

    class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K) : _pos3d(pos), _K(K) {}

        virtual void computeError() override 
        {
                const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
                Sophus::SE3d T = v->estimate();
                Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
                pos_pixel /= pos_pixel[2];
                _error = _measurement - pos_pixel.head<2>();
        }

        virtual void linearizeOplus() override 
        {
            const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
            Sophus::SE3d T = v->estimate();
            Eigen::Vector3d pos_cam = T * _pos3d;
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double cx = _K(0, 2);
            double cy = _K(1, 2);
            double X = pos_cam[0];
            double Y = pos_cam[1];
            double Z = pos_cam[2];
            double Z2 = Z * Z;
            _jacobianOplusXi
                << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
                0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
        }

        virtual bool read(std::istream &in) override {}
        virtual bool write(std::ostream &out) const override {}

    private:
        Eigen::Vector3d _pos3d;
        Eigen::Matrix3d _K;
    };

    class PoseEstimation3D2D
    {
    public:
        // BA by g2o
        using VecVector2d = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;
        using VecVector3d = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

        void RunDemo(const std::string& iamge_1, const std::string& iamge_depth_1, 
                    const std::string& iamge_2, const std::string& iamge_depth_2);

    private:
        void FindFeatureMatches(
                const cv::Mat &img_1, const cv::Mat &img_2,
                std::vector<cv::KeyPoint> &keypoints_1,
                std::vector<cv::KeyPoint> &keypoints_2,
                std::vector<cv::DMatch> &matches);

        void BundleAdjustmentG2O(
            const VecVector3d &points_3d,
            const VecVector2d &points_2d,
            const cv::Mat &K, 
            Sophus::SE3d &pose);

        // BA by gauss-newton
        void BundleAdjustmentGaussNewton(
            const VecVector3d &points_3d,
            const VecVector2d &points_2d,
            const cv::Mat &K,
            Sophus::SE3d &pose);

        // 像素坐标转相机归一化坐标
        cv::Point2d Pixel2Cam(const cv::Point2d &p, const cv::Mat &K);
    };


.. code-block:: c++

    void PoseEstimation3D2D::PoseEstimation3D2D::RunDemo(
    const std::string& iamge_1, const std::string& iamge_depth_1, 
    const std::string& iamge_2, const std::string& iamge_depth_2)
    {
        //-- 读取图像
        cv::Mat img_1 = cv::imread(iamge_1, cv::IMREAD_COLOR);
        cv::Mat img_2 = cv::imread(iamge_2, cv::IMREAD_COLOR);
        assert(img_1.data && img_2.data && "Can not load images!");

        std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
        std::vector<cv::DMatch> matches;
        FindFeatureMatches(img_1, img_2, keypoints_1, keypoints_2, matches);
        std::cout << "一共找到了" << matches.size() << "组匹配点" << std::endl;

        // 建立3D点
        cv::Mat d1 = cv::imread(iamge_depth_1, cv::IMREAD_UNCHANGED);       // 深度图为16位无符号数，单通道图像
        cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
        std::vector<cv::Point3f> pts_3d;
        std::vector<cv::Point2f> pts_2d;
        for (cv::DMatch m : matches) {
            ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
            if (d == 0)   // bad depth
            continue;
            float dd = d / 5000.0;
            cv::Point2d p1 = Pixel2Cam(keypoints_1[m.queryIdx].pt, K);
            pts_3d.push_back(cv::Point3f(p1.x * dd, p1.y * dd, dd));
            pts_2d.push_back(keypoints_2[m.trainIdx].pt);
        }

        std::cout << "3d-2d pairs: " << pts_3d.size() << std::endl;

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        cv::Mat r, t;
        cv::solvePnP(pts_3d, pts_2d, K, cv::Mat(), r, t, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
        cv::Mat R;
        cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << std::endl;

        std::cout << "R=" << std::endl << R << std::endl;
        std::cout << "t=" << std::endl << t << std::endl;

        VecVector3d pts_3d_eigen;
        VecVector2d pts_2d_eigen;
        for (size_t i = 0; i < pts_3d.size(); ++i) 
        {
            pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
            pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x, pts_2d[i].y));
        }

        std::cout << "calling bundle adjustment by gauss newton" << std::endl;
        Sophus::SE3d pose_gn;
        t1 = std::chrono::steady_clock::now();
        BundleAdjustmentGaussNewton(pts_3d_eigen, pts_2d_eigen, K, pose_gn);
        t2 = std::chrono::steady_clock::now();
        time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "solve pnp by gauss newton cost time: " << time_used.count() << " seconds." << std::endl;

        std::cout << "calling bundle adjustment by g2o" << std::endl;
        Sophus::SE3d pose_g2o;
        t1 = std::chrono::steady_clock::now();
        BundleAdjustmentG2O(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
        t2 = std::chrono::steady_clock::now();
        time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << std::endl;
    }


    void PoseEstimation3D2D::FindFeatureMatches(
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
        // BFMatcher matcher ( NORM_HAMMING );
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

    void PoseEstimation3D2D::BundleAdjustmentG2O(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const cv::Mat &K, 
        Sophus::SE3d &pose)
    {
        // 构建图优化，先设定g2o
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;  // pose is 6, landmark is 3
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
        // 梯度下降方法，可以从GN, LM, DogLeg 中选
        auto solver = new g2o::OptimizationAlgorithmGaussNewton(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;   // 图模型
        optimizer.setAlgorithm(solver);   // 设置求解器
        optimizer.setVerbose(true);       // 打开调试输出

        // vertex
        VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
        vertex_pose->setId(0);
        vertex_pose->setEstimate(Sophus::SE3d());
        optimizer.addVertex(vertex_pose);

        // K
        Eigen::Matrix3d K_eigen;
        K_eigen <<
                K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
            K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
            K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

        // edges
        int index = 1;
        for (size_t i = 0; i < points_2d.size(); ++i) {
            auto p2d = points_2d[i];
            auto p3d = points_3d[i];
            EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(p2d);
            edge->setInformation(Eigen::Matrix2d::Identity());
            optimizer.addEdge(edge);
            index++;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "optimization costs time: " << time_used.count() << " seconds." << std::endl;
    std:: cout << "pose estimated by g2o =\n" << vertex_pose->estimate().matrix() << std::endl;
        pose = vertex_pose->estimate();
    }

    void PoseEstimation3D2D::BundleAdjustmentGaussNewton(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const cv::Mat &K,
        Sophus::SE3d &pose)
    {
        typedef Eigen::Matrix<double, 6, 1> Vector6d;
        const int iterations = 10;
        double cost = 0, lastCost = 0;
        double fx = K.at<double>(0, 0);
        double fy = K.at<double>(1, 1);
        double cx = K.at<double>(0, 2);
        double cy = K.at<double>(1, 2);

        for (int iter = 0; iter < iterations; iter++) {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < points_3d.size(); i++) {
            Eigen::Vector3d pc = pose * points_3d[i];
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            Eigen::Vector2d proj(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);

            Eigen::Vector2d e = points_2d[i] - proj;

            cost += e.squaredNorm();
            Eigen::Matrix<double, 2, 6> J;
            J << -fx * inv_z,
            0,
            fx * pc[0] * inv_z2,
            fx * pc[0] * pc[1] * inv_z2,
            -fx - fx * pc[0] * pc[0] * inv_z2,
            fx * pc[1] * inv_z,
            0,
            -fy * inv_z,
            fy * pc[1] * inv_z2,
            fy + fy * pc[1] * pc[1] * inv_z2,
            -fy * pc[0] * pc[1] * inv_z2,
            -fy * pc[0] * inv_z;

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        Vector6d dx;
        dx = H.ldlt().solve(b);

        if (isnan(dx[0])) {
            std::cout << "result is nan!" << std::endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            std::cout << "cost: " << cost << ", last cost: " << lastCost << std::endl;
            break;
        }

        // update your estimation
        pose = Sophus::SE3d::exp(dx) * pose;
        lastCost = cost;

        std::cout << "iteration " << iter << " cost=" << std::setprecision(12) << cost << std::endl;
        if (dx.norm() < 1e-6) {
            // converge
            break;
        }
        }

        std::cout << "pose by g-n: \n" << pose.matrix() << std::endl;
    }

    cv::Point2d PoseEstimation3D2D::Pixel2Cam(const cv::Point2d &p, const cv::Mat &K)
    {
        return cv::Point2d
        {
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
        };
    }


运行结果

.. code-block:: bash

    [bin] ./xslam.opencv.geometry_transform.pose_estimation_3d2d_test

    Running main() from gmock_main.cc
    [==========] Running 1 test from 1 test suite.
    [----------] Global test environment set-up.
    [----------] 1 test from PoseEstimation3D2D
    [ RUN      ] PoseEstimation3D2D.pose_estimation_2d
    -- Max dist : 94.000000 
    -- Min dist : 4.000000 
    一共找到了79组匹配点
    3d-2d pairs: 75
    solve pnp in opencv cost time: 0.00233683 seconds.
    R=
    [0.9979059095501289, -0.05091940089111062, 0.03988747043647115;
    0.04981866254254162, 0.9983623157438141, 0.02812094175381178;
    -0.04125404886071617, -0.02607491352889358, 0.9988083912027663]
    t=
    [-0.1267821389556796;
    -0.008439496817594587;
    0.06034935748886031]
    calling bundle adjustment by gauss newton
    iteration 0 cost=40517.7576706
    iteration 1 cost=410.547029116
    iteration 2 cost=299.76468142
    iteration 3 cost=299.763574327
    pose by g-n: 
    0.997905909549  -0.0509194008562   0.0398874705187   -0.126782139096
    0.049818662505    0.998362315745   0.0281209417649 -0.00843949683874
    -0.0412540489424  -0.0260749135374    0.998808391199   0.0603493575229
                    0                 0                 0                 1
    solve pnp by gauss newton cost time: 8.796e-05 seconds.
    calling bundle adjustment by g2o
    iteration= 0	 chi2= 410.547029	 time= 1.4597e-05	 cumTime= 1.4597e-05	 edges= 75	 schur= 0
    iteration= 1	 chi2= 299.764681	 time= 8.711e-06	 cumTime= 2.3308e-05	 edges= 75	 schur= 0
    iteration= 2	 chi2= 299.763574	 time= 8.659e-06	 cumTime= 3.1967e-05	 edges= 75	 schur= 0
    iteration= 3	 chi2= 299.763574	 time= 7.967e-06	 cumTime= 3.9934e-05	 edges= 75	 schur= 0
    iteration= 4	 chi2= 299.763574	 time= 7.965e-06	 cumTime= 4.7899e-05	 edges= 75	 schur= 0
    iteration= 5	 chi2= 299.763574	 time= 2.7984e-05	 cumTime= 7.5883e-05	 edges= 75	 schur= 0
    iteration= 6	 chi2= 299.763574	 time= 2.0946e-05	 cumTime= 9.6829e-05	 edges= 75	 schur= 0
    iteration= 7	 chi2= 299.763574	 time= 8.356e-06	 cumTime= 0.000105185	 edges= 75	 schur= 0
    iteration= 8	 chi2= 299.763574	 time= 8.105e-06	 cumTime= 0.00011329	 edges= 75	 schur= 0
    iteration= 9	 chi2= 299.763574	 time= 8.142e-06	 cumTime= 0.000121432	 edges= 75	 schur= 0
    optimization costs time: 0.000459426 seconds.
    pose estimated by g2o =
        0.99790590955  -0.0509194008911   0.0398874704367   -0.126782138956
    0.0498186625425    0.998362315744   0.0281209417542 -0.00843949681823
    -0.0412540488609  -0.0260749135293    0.998808391203   0.0603493574888
                    0                 0                 0                 1
    solve pnp by g2o cost time: 0.000722355 seconds.
    [       OK ] PoseEstimation3D2D.pose_estimation_2d (1784 ms)
    [----------] 1 test from PoseEstimation3D2D (1784 ms total)

    [----------] Global test environment tear-down
    [==========] 1 test from 1 test suite ran. (1784 ms total)
    [  PASSED  ] 1 test.


参考源码：

.. NOTE::

    * pose_estimation_3d2d_test.cpp
    * pose_estimation_3d2d.cpp
    * pose_estimation_3d2d.h