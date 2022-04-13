.. highlight:: c++

.. default-domain:: cpp

===========
Image Basic
===========

1 Read Image
==================

.. code-block:: c++

    cv::Mat cv::imread(const std::string &filename, int flags = IMREAD_COLOR)

    enum cv::ImreadModes 
    {
        IMREAD_UNCHANGED,			//-1    使图像保持原样输出  
        IMREAD_GRAYSCALE,			//0     把图像转成单通道的灰度图输出
        IMREAD_COLOR ,				//1   	//把图像转成三通道的rgb图输出
        IMREAD_ANYDEPTH, 			//2     //If set, return 16-bit/32-bit image when the input has the corresponding depth, otherwise convert it to 8-bit.
        IMREAD_ANYCOLOR	,			//4     //以任何可能的颜色格式读取图像
        IMREAD_LOAD_GDAL, 			//8 	//use the gdal driver for loading the image
        IMREAD_REDUCED_GRAYSCALE_2,	//16	//输出单通道灰度图，并且将图像缩小为原来的1/2
        IMREAD_REDUCED_COLOR_2 ,	//17    //输出三通道的rgb图，并且缩小图像到原来的1/2
        IMREAD_REDUCED_GRAYSCALE_4, //32    //单通道  1/4
        IMREAD_REDUCED_COLOR_4 ,	//33	//三通道  1/4
        IMREAD_REDUCED_GRAYSCALE_8, //64	//单通道  1/8
        IMREAD_REDUCED_COLOR_8 ,	//65	//三通道  1/8
        IMREAD_IGNORE_ORIENTATION 	//128	//do not rotate the image according to EXIF's orientation flag. 
    }

demo调用, 源码 [#f1]_ [#f2]_ [#f3]_：

.. code-block:: c++

    TEST(ReadImage, read_image_test)
    {
        std::string filename = GetOpenCVDatasetDirectory() + "/0001_tian_an_man.jpeg";
        ReadImage demo;
        demo.Read(filename);
    }

函数使用：

.. code-block:: c++

    void ReadImage::Read(const std::string& filename)
    {
        cv::Mat image = cv::imread(filename);

        if (image.data == nullptr) {
            std::cout << "Load image error." << std::endl;
            exit(-1);
        }

        cv::imshow("Opencv read image", image);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }


.. rubric:: Footnotes

.. [#f1] `read_image.cpp
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/opencv/read_image.cpp>`_
.. [#f2] `read_image.h
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/opencv/read_image.h>`_
.. [#f3] `read_image_test.cpp
    <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/opencv/read_image_test.cpp>`_

#. 图像宽,高,通道信息
#. 图像的类型
#. 遍历图像
#. cv::Mat的拷贝
#. 图像修改

demo调用, 源码 [#f4]_ [#f5]_ [#f6]_：

.. code-block:: c++

    TEST(ImageBasic, ImageOperator)
    {
        std::string filename = GetOpenCVDatasetDirectory() + "/0006_ubuntu.png";
        ImageBasic demo;
        demo.RunDemo(filename);
    }

函数使用：

.. code-block:: c++

    void ImageBasic::RunDemo(const std::string& filename)
    {
        cv::Mat image = cv::imread(filename); //cv::imread函数读取指定路径下的图像
        // 判断图像文件是否正确读取
        if (image.data == nullptr) { //数据不存在,可能是文件不存在
            std::cerr << "文件: " << filename << "不存在." << std::endl;
            return;
        }

        // 文件顺利读取, 首先输出一些基本信息
        std::cout << "图像宽为: "  << image.cols
                << ",高为: "    << image.rows
                << ",通道数为: " << image.channels()
                << std::endl;

        cv::imshow("image", image);    // 用cv::imshow显示图像
        cv::waitKey(0);                  // 暂停程序,等待一个按键输入

        // 判断image的类型
        if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
            // 图像类型不符合要求
            std::cout << "请输入一张彩色图或灰度图." << std::endl;
            return;
        }

        // 遍历图像, 请注意以下遍历方式亦可使用于随机像素访问
        // 使用 std::chrono 来给算法计时
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        for (size_t y = 0; y < image.rows; y++) {
            // 用cv::Mat::ptr获得图像的行指针
            unsigned char *row_ptr = image.ptr<unsigned char>(y);  // row_ptr是第y行的头指针
            for (size_t x = 0; x < image.cols; x++) {
                // 访问位于 x,y 处的像素
                unsigned char *data_ptr = &row_ptr[x * image.channels()]; // data_ptr 指向待访问的像素数据
                // 输出该像素的每个通道,如果是灰度图就只有一个通道
                for (int c = 0; c != image.channels(); c++) {
                    unsigned char data = data_ptr[c]; // data为I(x,y)第c个通道的值
                }
            }
        }
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used =
                std::chrono::duration_cast<std::chrono::duration <double>>(t2 - t1);
        std::cout << "遍历图像用时： " << time_used.count() << " 秒。" << std::endl;

        // 关于 cv::Mat 的拷贝
        // 直接赋值并不会拷贝数据
        cv::Mat image_another = image;
        // 修改 image_another 会导致 image 发生变化
        image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // 将左上角100*100的块置零
        cv::imshow("image", image);
        cv::waitKey(0);

        // 使用clone函数来拷贝数据
        cv::Mat image_clone = image.clone();
        image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
        cv::imshow("image", image);
        cv::imshow("image_clone", image_clone);
        cv::waitKey(0);

        // 对于图像还有很多基本的操作,如剪切,旋转,缩放等,限于篇幅就不一一介绍了,
        // 请参看OpenCV官方文档查询每个函数的调用方法.
        cv::destroyAllWindows();
    }

.. rubric:: Footnotes

.. [#f4] `image_basic.cpp
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/opencv/image_basic.cpp>`_
.. [#f5] `image_basic.h
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/opencv/image_basic.h>`_
.. [#f6] `image_basic_test.cpp
    <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/opencv/image_basic_test.cpp>`_


2 Paint Shape
==================

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

demo调用, 源码 [#f7]_ [#f8]_ [#f9]_：

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

.. [#f7] `paint_shape.cpp
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/opencv/paint_shape.cpp>`_
.. [#f8] `paint_shape.h
   <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/opencv/paint_shape.h>`_
.. [#f9] `paint_shape_test.cpp
    <https://github.com/quanduyong/LTSLAM/blob/main/xslam/xslam/opencv/paint_shape_test.cpp>`_

