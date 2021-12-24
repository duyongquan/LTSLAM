.. highlight:: c++

.. default-domain:: cpp

===========
Paint Shape
===========

Opencv API : 

.. code-block:: c++ 
        
        void rectangle(Mat& img, Point pt1,Point pt2,const Scalar& color, 
            int thickness=1, int lineType=8, int shift=0)
 

Opencv API : 

 .. code-block:: c++ 

        void cvLine( CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color, 
            int thickness=1, int line_type=8, int shift=0 );

.. NOTE::

    第一个参数img：要划的线所在的图像

    第二个参数pt1：直线起点

    第二个参数pt2：直线终点

    第三个参数color：直线的颜色 e.g:Scalor(0,0,255)

    第四个参数thickness=1：线条粗细

    第五个参数line_type=8,
    
        8 - 8-connected line（8邻接)连接 线。

        4 - 4-connected line(4邻接)连接线。

        CV_AA - antialiased 线条。

    第六个参数：坐标点的小数点位数。

demo调用, 源码 [#f1]_ [#f2]_ [#f3]_：

.. code-block:: c++

   TEST(PaintShape, shape)
    {
        LOG(INFO) << "Run PaintShape demos ...";
        // OpenCV
        PaintShape demo;
        demo.RunDemo();
    }

函数使用：

.. code-block:: c++

    void PaintShape::RunDemo()
    {
        // 1 line
        PaintLine();

        // 2 circle
        PaintCircle();

        // Sets the
        PaintRectangle();

        // 4 rectangle
        PaintRectangle();
    }

    void PaintShape::PaintLine()
    {
        // 创建黑色的图像
        cv::Mat image = cv::Mat(512, 512, CV_8UC3);

        // 绘制一条厚度为5的蓝色对角线
        cv::line(image, cv::Point(0, 0), cv::Point(511, 511), cv::Scalar(255, 0, 0), 5);

        cv::imshow("PaintShape", image);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }   

    void PaintShape::PaintCircle()
    {
        // 创建黑色的图像
        cv::Mat image = cv::Mat(512, 512, CV_8UC3);

        // 绘制一条厚度为5的蓝色对角线
        cv::circle(image, cv::Point(447, 63), 63, cv::Scalar(0, 0, 255), -1);

        cv::imshow("PaintShape", image);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    void PaintShape::PaintRectangle()
    {
        // 创建黑色的图像
        cv::Mat image = cv::Mat(512, 512, CV_8UC3);

        // 绘制一条厚度为5的蓝色对角线
        cv::rectangle(image, cv::Point(384, 0), cv::Point(510, 128), cv::Scalar(0, 255, 0), 3);

        cv::imshow("PaintShape", image);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }


.. rubric:: Footnotes

.. [#f1] `paint_shape.cpp
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/opencv/paint_shape.cpp>`_
.. [#f2] `paint_shape.h
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/opencv/paint_shape.h>`_
.. [#f3] `paint_shape_test.cpp
    <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/opencv/paint_shape_test.cpp>`_
