# LTSLAM
## 1 Introduction

lean slam step by step, you can online learn [LTSLAM](https://ltslam-doc.readthedocs.io/en/latest/index.html)

project directory:

* xlsam
* xlsam_ros

download LTSLAM source:

```shell
git clone https://github.com/quanduyong/LTSLAM.git
```

## 2 Install dependency

### 2.1 Generate study documents

* 安装Sphinx

  ```shell
  pip install -U sphinx
  ```

* 安装主题

  ```shell
  pip install sphinx_rtd_theme
  ```

* 安装markdown插件

  ```shell
  pip install recommonmark
  pip install sphinx_markdown_tables
  ```

### 2.2 Third party library

<font  color='green'>(推荐)</font>我已经提供了第三方库源码文件3rdparty目录下，执行以下命令安装，<font  color='red'>不然会出现版本不匹配问题</font>：

  ```shell
  mkdir build && cd build
  cmake ..
  make -j6 
  sudo make install
  ```

​     <font  color='red'>(不推荐)</font>如果你想自己源码安装第三方库请使用以下步骤：

* 安装Eigen

  ```
  git clone https://gitlab.com/libeigen/eigen.git
  cd eigen
  git checkout 3.3.8
  mkdir build && cd build
  cmake ..
  make -j6 
  sudo make install
  ```

* 安装Sophus

  ```shell
  git clone https://github.com/strasdat/Sophus.git
  cd Sophus
  mkdir build && cd build
  cmake ..
  make -j6 
  sudo make install
  ```

* 安装abseil 

  在abseil项目的CMakeLists.txt的project(absl LANGUAGES CXX) 下添加以下命令：

  `add_compile_options(-fPIC)`

  ```shell
  git clone https://github.com/abseil/abseil-cpp.git
  cd abseil-cpp
  mkdir build && cd build
  cmake ..
  make -j6 
  sudo make install
  ```

* 安装 DBow3

  ```
  git clone https://github.com/rmsalinas/DBow3.git
  cd abseil-cpp
  mkdir build && cd build
  cmake ..
  make -j6 
  sudo make install  
  ```

* 安装googletest

  ```
  git clone https://github.com/google/googletest.git
  cd googletest
  mkdir build && cd build
  cmake ..
  make -j6 
  sudo make install  
  ```

* 安装g2o

  ```
  git clone https://github.com/rmsalinas/DBow3.git
  cd abseil-cpp
  git checkout 9b41a4ea
  mkdir build && cd build
  cmake ..
  make -j6 
  sudo make install  
  ```

* 安装opencv & opencv_contrib (4.5-x)

  ```
  git clone https://github.com/opencv/opencv.git
  git clone https://github.com/opencv/opencv_contrib.git
  
  cd opencv
  mkdir build && cd build
  cmake-gui .. # 选择extra module opencv_contrib
  make -j6 
  sudo make install
  ```

## 3  工程编译

```shell
cd LTSLAM
mkdir build
cd build 
cmake ..
make -j6
```

## 4 如何运行工程demo

```

```

