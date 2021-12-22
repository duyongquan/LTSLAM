# slam_tutorial

#### 介绍
SLAM tutorial for all robot.

<img src='./docs/images/slam_project.jpg'>

#### 目录结构

* cmake : 工程需要的CMake函数
* data ： 运行demo的数据集
  * oepncv : 图片数据
  * ceres ： 数据集

* docs ： 记录文档
* slam
  * ceres ： 官方例子
  * eigen ： matrix例子
  * example ： 可执行的bin文件
  * kalman_filter ： jupyter notebook格式
  * pangolin : 3D可视化
  * sophus ： 李群李代数
  * opencv ： 简单的demo
  * scipts ： 运行可视化脚本



#### 第三方依赖

安装ROS，大部分库就可使用

* boost
* opencv



ROS包含(本人安装的版本opencv4-5.2， opencv-contrilib-4.5.2)

```shell
# opencv4-5.2
https://github.com/opencv/opencv.git

# opencv-contrilib-4.5.2
https://github.com/opencv/opencv_contrib.git
```



* eigen

```shell
git clone https://gitlab.com/libeigen/eigen.git
```



* sophus

```shell
https://github.com/strasdat/Sophus.git
```



* Pangolin

```shell
https://github.com/stevenlovegrove/Pangolin.git
```



* g2o

```shell
https://github.com/RainerKuemmerle/g2o.git
```



* ceres solver

```shell
https://github.com/ceres-solver/ceres-solver.git
```



#### 安装教程

```shell
git clone https://gitee.com/quanduyong/slam_tutorial.git
cd slam_tutorial
mkdir build; cd build; cmake ..
make -j6
```



#### IDE Clion & VSCode

* 推荐ubuntu
* windows环境下暂时不支持



#### 郑重声明

本工程开源学习：[哔哩哔哩视频](https://space.bilibili.com/478832908?spm_id_from=333.788.b_765f7570696e666f.2)

<img src='./docs/images/wei_chat.jpg' width='50%' heigh='50%'>
