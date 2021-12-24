.. highlight:: c++

.. default-domain:: cpp

====================
Fundamentals Matrix
====================

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
