.. highlight:: c++

.. default-domain:: cpp

==================
Feature Detection
==================

1 理解特征
==================

寻找独特的，易于跟踪和比较的特定模板或特定特征。如果我们对这种特征进行定义，可能会发现很难用语言来表达它，
但是我们知道它们是什么。如果有人要求你指出一项可以在多张图像中进行比较的良好特征，则可以指出其中一项。

因此，我们的一个基本问题扩展到更多，但变得更加具体。这些特征是什么？（答案对于计算机也应该是可以理解的。）

很难说人类如何发现这些特征。这已经在我们的大脑中进行了编码。但是，如果我们深入研究某些图片并搜索不同的模板，
我们会发现一些有趣的东西。例如，看以下的图片：

.. figure:: ./images/feature_building.jpg
   :align: center

2 Harris角点检测
==================

它基本上找到了 :math:`(u，v)` 在所有方向上位移的强度差异。表示如下：

.. math:: 

    E(u, v) = \sum_{x,y} w(x, y) [I(x+u, y+v) - I(x, y)]

窗口函数要么是一个矩形窗口,要么是高斯窗口,它在下面赋予了值。

我们必须最大化这个函数 :math:`E(u,v)` 用于角检测。这意味着,我们必须最大化第二个项。
将泰勒扩展应用于上述方程,并使用一些数学步骤(请参考任何你喜欢的标准文本书),
我们得到最后的等式:

.. math::

    E(u, v) \simeq   
    \begin{bmatrix}
        u & v
    \end{bmatrix} M 
    \begin{bmatrix}
        u \\
        v
    \end{bmatrix}

其中

.. math::

    M = \sum_{x,y} w(x, y)
     \begin{bmatrix}
        I_{x}I_{x} & I_{x}I_{y} \\
        I_{x}I_{y} & I_{y}I_{y} 
    \end{bmatrix}

在此，:math:`Ix` 和 :math:`Iy`分别是在x和y方向上的图像导数。可以使用 **cv::Sobel()** 轻松找到.


然后是主要部分。之后，他们创建了一个分数，基本上是一个等式，
它将确定一个窗口是否可以包含一个角。

.. math::

    R = det(M) - k(trace(M))^2

其中
   * :math:`det(M)=λ1λ2`

   * :math:`trace(M)=λ1+λ2`

   * :math:`λ1` and :math:`λ2` 是 :math:`M` 的特征值

可以用如下图来表示：

.. figure:: ./images/harris_region.jpg
   :align: center

因此，Harris Corner Detection的结果是具有这些分数的灰度图像。
合适的阈值可为您提供图像的各个角落。我们将以一个简单的图像来完成它。

Opencv C++ API:

.. code-block:: c++

    void cornerHarris( InputArray src, OutputArray dst, int block Size, 
        int ksize, double k, int borderType = BORDER_DEFAULT)

.. NOTE:: 

    * InputArray类型的src，输入图像，即原图像，填Mat类型即可，且需要为单通道8位或者浮点型图像
    * OutputArray类型的dst，函数调用后的运算结果存在这里，即这个参数用于存放Harris角点检测的输出结果，和原图片有一样的尺寸和类型
    * int类型的blockSize，表示邻域的大小，更多详细信息在cornerEigenValsAndVecs()中讲到
    * int类型的ksize，表示Sobel()算子的孔径的大小
    * double类型的k，Harris参数
    * int类型的borderType，图像像素的边界模式。注意它有默认值BORDER_DEFAULT

demo调用, 源码 

.. code-block:: c++

    TEST(CornerHarris, cornerDetect)
    {
        std::string filename = GetOpenCVDatasetDirectory() + "/0002_chessboard.jpeg";
        CornerHarris demo;
        demo.CornerDetect(filename);
    }

函数使用：

.. code-block:: c++
        
    void CornerHarris::CornerDetect(const std::string& filename)
    {
        // 1 read a image
        cv::Mat image = cv::imread(filename);
        if (image.data == nullptr) {
            std::cout << "Load image error." << std::endl;
            exit(-1);
        }

        // 2 convert to gray
        cv::Mat gray;
        cv::cvtColor(image, gray,cv::COLOR_BGR2GRAY);

        // 3 cornerHarris角点检测
        // 进行角点检测
        // 领域大小为 2
        // sobel 算子孔径 3
        // harris 参数
        cv::Mat dstImage;       //目标图
        cv::Mat normImage;      //归一化后的图
        cv::Mat scaledImage;    //线性变换后的八位无符号整型的图

        //置零当前需要显示的两幅图，即清除上一次调用此函数时他们的值
        dstImage = cv::Mat::zeros(image.size(), CV_32FC1 );
        cv::cornerHarris(gray, dstImage, 2, 3, 0.04, cv::BORDER_DEFAULT );

        // 归一化与转换
        cv::normalize( dstImage, normImage, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
        convertScaleAbs( normImage, scaledImage );      //将归一化后的图线性变换成8位无符号整型

        // 4、进行绘制
        // 将检测到的，且符合阈值条件的角点绘制出来
        int corner_count = 0;
        for( int j = 0; j < normImage.rows ; j++ )
            for( int i = 0; i < normImage.cols; i++ )
            {
                if( (int) normImage.at<float>(j,i) > 80 )    //  设定阈值
                {
                    cv::circle(image, cv::Point( i, j ), 6,  cv::Scalar(0,255,5), 2, 1, 0 );
                }
            }

        // 5 显示最终效果
        cv::imshow("CornerHarris Corner Detected", image);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

运行结果

.. code-block:: bash

    [bin] ./xslam.opencv.feature_detection.corner_harris_test

.. figure:: ./images/harris_result.png
   :align: center

参考源码：

.. NOTE::

    * corner_harris_test.h
    * corner_harris.cpp
    * corner_harris.h


3 Fast ORB角点检测
==================

Opencv C++ API:

.. code-block:: c++

    Ptr<FastFeatureDetector> create(int threshold=10,
        bool nonmaxSuppression=true,int type=FastFeatureDetector::TYPE_9_16 );

.. NOTE:: 

    * threshold: 阈值
    * nonmaxSuppression: 非极大值抑制
    * type: 邻域类型

demo调用, 源码

.. code-block:: c++

    TEST(FastFeature, cornerDetect)
    {
        std::string filename = GetOpenCVDatasetDirectory() + "/0002_chessboard.jpeg";
        FastFeature demo;
        demo.CornerDetect(filename);
    }


函数使用：

.. code-block:: c++

    void FastFeature::CornerDetect(const std::string& filename)
    {
        // 1 read a image
        cv::Mat image = cv::imread(filename);
        if (image.data == nullptr) {
            std::cout << "Load image error." << std::endl;
            exit(-1);
        }

        // 2 convert to gray
        cv::Mat gray;
        cv::cvtColor(image, gray,cv::COLOR_BGR2GRAY);

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat dst = image.clone();
        cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(40);
        detector->detect(image,keypoints);
        drawKeypoints(dst, keypoints, dst, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);

        cv::imshow("FastFeature Corner Detected", dst);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }


运行结果

.. code-block:: bash

    [bin] ./xslam.opencv.feature_detection.fast_feature_detector_test


.. figure:: ./images/fast_orb_result.png
   :align: center

参考源码：

.. NOTE::

    * fast_feature_detector_test.h
    * fast_feature_detector.cpp
    * fast_feature_detector.h

4 SIFT角点检测
==================

demo调用, 源码

.. code-block:: c++

    TEST(SIFT, demo)
    {
        // 0008_roofs1.jpg
        std::string filename = GetOpenCVDatasetDirectory() + "/0002_chessboard.jpeg"; 
        SIFTFeature demo;
        demo.RunDemo(filename);
    }

函数使用：

.. code-block:: c++

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

运行结果

.. code-block:: bash

    [bin] ./xslam.opencv.feature_detection.SIFT_test 

.. figure:: ./images/sift_result.png
   :align: center

参考源码：

.. NOTE::

    * SIFT_test.h
    * SIFT.cpp
    * SIFT.h

5 shi tomasi角点检测
=====================

Opencv C++ API:

.. code-block:: c++

    void cv::goodFeaturesToTrack(InputArray _image, OutputArray _corners,
                              int maxCorners, double qualityLevel, double minDistance,
                              InputArray _mask, int blockSize,
                              bool useHarrisDetector, double harrisK)
    
                

.. NOTE:: 

    * _image：8位或32位浮点型输入图像，单通道
    * _corners：保存检测出的角点
    * maxCorners：角点数目最大值，如果实际检测的角点超过此值，则只返回前maxCorners个强角点
    * qualityLevel：角点的品质因子
    * minDistance：对于初选出的角点而言，如果在其周围minDistance范围内存在其他更强角点，则将此角点删除
    * _mask：指定感兴趣区，如不需在整幅图上寻找角点，则用此参数指定ROI
    * blockSize：计算协方差矩阵时的窗口大小
    * useHarrisDetector：指示是否使用Harris角点检测，如不指定，则计算shi-tomasi角点
    * harrisK：Harris角点检测需要的k值

demo调用, 源码

.. code-block:: c++

    TEST(ShiTomasi, GoodFeaturesToTrack)
    {
        std::string filename = GetOpenCVDatasetDirectory() + "/0002_chessboard.jpeg";
        ShiTomasi demo;
        demo.CornerDetect(filename);
    }

函数使用：

.. code-block:: c++

    void ShiTomasi::CornerDetect(const std::string& filename)
    {
        cv::Mat image = cv::imread(filename);
        if (image.data == nullptr) {
            std::cout << "Load image error." << filename << std::endl;
            exit(-1);
        }

        cv::Mat gray;
        cv::cvtColor(image, gray,cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        goodFeaturesToTrack(gray, corners, 100, 0.01,50, cv::Mat());
        for(int i = 0; i < corners.size(); i++) {
            // image，背景图
            // center，圆心
            // radius，半径
            // color，颜色
            // thickness，线粗细
            circle(image, corners[i],5,cv::Scalar(0,0,255),2);
        }

        imshow("Shi-Tomasi Corner Detected",image);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

运行结果

.. code-block:: bash

    [bin] ./xslam.opencv.feature_detection.shi_tomasi_test 

.. figure:: ./images/shi_tomasi.png
   :align: center


参考源码：

.. NOTE::

    * shi_tomasi_test.h
    * shi_tomasi.cpp
    * shi_tomasi.h

