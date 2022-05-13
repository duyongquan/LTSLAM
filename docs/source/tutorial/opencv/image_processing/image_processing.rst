.. highlight:: c++

.. default-domain:: cpp

==================
Image Processing
==================

1 改变颜色空间
==================


2 图像的几何变换
==================


3 图像阈值
==================


4 图像平滑
==================


5 形态学转换
==================


6 图像梯度
==================


7 Canny边缘检测
==================

由于边缘检测容易受到图像中噪声的影响, 因此第一步是使用5x5高斯滤波器消除图像中的噪声。

**查找图像的强度梯度**

然后使用Sobel核在水平和垂直方向上对平滑的图像进行滤波, 以在水平方向(Gx)和垂直方向(Gy)上获得一阶导数。
从这两张图片中，我们可以找到每个像素的边缘渐变和方向，如下所示：

.. math::

    Edge\_Gradient \; (G) = \sqrt{G_x^2 + G_y^2} \\ Angle \; (\theta) = \tan^{-1} \bigg(\frac{G_y}{G_x}\bigg)


渐变方向始终垂直于边缘。将其舍入为代表垂直，水平和两个对角线方向的四个角度之一。

Opencv C++ API:

.. code-block:: c++

    void Canny(InputArray image, OutputArray edges, 
        double threshold1, double threshold2, int apertureSize=3, bool L2gradient=false)

.. NOTE::

    * image: InputArray类型的image, 输入图像, Mat对象节课, 需为单通道8位图像。
    * edges: OutputArray类型的edges, 输出的边缘图，需要和输入图像有相同的尺寸和类型。
    * threshold1: double类型的threshold1, 第一个滞后性阈值。
    * threshold2: double类型的threshold2, 第二个滞后性阈值。
    * apertureSize: int类型的apertureSize, 表示算子的孔径的大小, 默认值时3.
    * L2gradient: bool类型的L2gradient, 一个计算图像梯度复制的标识, 默认false。

demo调用, 源码 

.. code-block:: c++

    TEST(Canny, shape)
    {
        LOG(INFO) << "Run Canny demos ...";
        
        // OpenCV
        std::string filename = GetOpenCVDatasetDirectory() + "/0011_canny.jpg";
        Canny demo;
        demo.RunDemo(filename);
    }


函数使用：

.. code-block:: c++

    void Canny::RunDemo(const std::string& filename)
    {
        // , grayImage;
        cv::Mat image = cv::imread(filename);
        if (image.data == nullptr) {
            std::cout << "Load image error." << std::endl;
            exit(-1);
        }

        cv::Mat grayImage;
        cv::Mat srcImage1 = image.clone();
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
        cv::Mat dstImage, edge;
    
        cv::blur(grayImage, grayImage, cv::Size(3,3));
        cv::Canny(grayImage, edge, 150, 100, 3);
    
        dstImage.create(srcImage1.size(), srcImage1.type());
        srcImage1.copyTo(dstImage, edge);

        cv::imshow("origin", image);
        cv::imshow("canny", dstImage);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

运行结果

.. code-block:: bash

    [bin] ./xslam.opencv.image_processing.canny_test

.. figure:: ./images/canny.png
   :align: center

参考源码：

.. NOTE::

    * canny_test.h
    * canny.cpp
    * canny.h

8 图像金字塔
==================


9 轮廓
==================


10 直方图
==================

11 傅里叶变换
==================

12 模板匹配
==================


13 霍夫线变换
==================

14 霍夫圆变换
==================

15 图像分割与Watershed算法
==========================

16 交互式前景提取使用GrabCut算法
================================


