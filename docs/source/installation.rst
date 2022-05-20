.. _chapter-installation:

=======
安装
=======

获取源码
========

.. _section-source:

你可以使用最新 `代码
<https://github.com/quanduyong/LTSLAM.git>`_ .

.. code-block:: bash

       git clone https://github.com/quanduyong/LTSLAM.git

.. _section-dependencies:

依赖
====

X-SLAM工程依赖一系列c++开源库。

- `Eigen <http://eigen.tuxfamily.org/index.php?title=Main_Page>`_
  3.3.7 or later **strongly** recommended, 3.3.7 or later **required**.

- `CMake <http://www.cmake.org>`_ 2.8.0 or later.
  **Required on all platforms except for Android.**

- `glog <https://github.com/google/glog>`_ 0.5 or later. **Recommended**

- `gflags <https://github.com/gflags/gflags>`_. Needed to build examples and tests.

- `googletest <https://github.com/google/googletest.git>`_. Needed to build examples and tests.

- `SuiteSparse
  <http://faculty.cse.tamu.edu/davis/suitesparse.html>`_. Needed for
  solving large sparse linear systems. **Optional; strongly recomended
  for large scale bundle adjustment**

- `CXSparse <http://faculty.cse.tamu.edu/davis/suitesparse.html>`_.
  Similar to ``SuiteSparse`` but simpler and slower. CXSparse has
  no dependencies on ``LAPACK`` and ``BLAS``. This makes for a simpler
  build process and a smaller binary. **Optional**

- `opencv-4.5.2 <https://github.com/opencv/opencv.git>`_. You will be build examples and tests or not.

- `opencv_contrib-4.5.2 <https://github.com/opencv/opencv_contrib.git>`_. You will be build examples and tests or not.

- `g2o <https://github.com/RainerKuemmerle/g2o.git>`_. You will bebuild examples and tests or not.
   
- `ceres solver <https://github.com/ceres-solver/ceres-solver.git>`_. You will be build examples and tests or not.

- `pangolin <https://github.com/stevenlovegrove/Pangolin.git>`_. You will be build examples and tests or not.

- `sophus <https://github.com/strasdat/Sophus.git>`_. You will be build examples and tests or not.

ROS1
====

ubuntu18.04安装ROS Melodic最详细配置

设置软件源：

.. code-block:: bash

   sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'


设置最新的密钥：

.. code-block:: bash

      sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654


安装：

.. code-block:: bash

      sudo apt-get update
      sudo apt-get install ros-melodic-desktop-full
      sudo apt-get install ros-melodic-rqt*


初始化rosdep:

.. code-block:: bash

      sudo rosdep init
      rosdep update


安装rosinstall：

.. code-block:: bash

      sudo apt-get install python-rosinstall
 

加载环境设置文件：

.. code-block:: bash

      source /opt/ros/melodic/setup.bash

设置环境变量：

.. code-block:: bash

      # Set ROS Network
      #ifconfig查看你的电脑ip地址
      export ROS_HOSTNAME=192.168.89.135
      export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311


.. _section-linux:

ROS2(Galactic)
==============

**Set locale**

.. code-block:: bash

      locale  # check for UTF-8

      sudo apt update && sudo apt install locales
      sudo locale-gen en_US en_US.UTF-8
      sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
      export LANG=en_US.UTF-8

      locale  # verify settings


**Setup Sources**

You will need to add the ROS 2 apt repositories to your system. To do so, first authorize our GPG key with apt like this:

.. code-block:: bash

      sudo apt update && sudo apt install curl gnupg2 lsb-release
      sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

.. code-block:: bash

      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

**Install ROS 2 packages**

.. code-block:: bash

      sudo apt update
      sudo apt install ros-galactic-desktop



Linux
=====

We will use `Ubuntu18.04 <http://www.ubuntu.com>`_ as our example linux
distribution.


Start by installing all the dependencies.

CMake tools and some libraries

.. code-block:: bash

     # CMake
     sudo apt-get install cmake
     # google-glog + gflags
     sudo apt-get install libgoogle-glog-dev
     # BLAS & LAPACK
     sudo apt-get install libatlas-base-dev
     # suitesparse
     sudo apt-get install libsuitesparse-dev
    
glog

.. code-block:: bash

      
      git clone https://github.com/google/glog
      mkdir build
      cd build
      make -j6
      sudo make install

gflags

.. code-block:: bash

      
      git clone https://github.com/gflags/gflags
      mkdir build
      cd build
      make -j6
      sudo make install

googletest

.. code-block:: bash

      
      git clone https://github.com/google/googletest.git
      mkdir build
      cd build
      make -j6
      sudo make install

ceres solver

.. code-block:: bash

      
      git clone https://github.com/ceres-solver/ceres-solver.git
      mkdir build
      cd build
      make -j6
      sudo make install

pangolin

.. code-block:: bash

      
      git clone https://github.com/stevenlovegrove/Pangolin.git
      mkdir build
      cd build
      make -j6
      sudo make install


g2o

.. code-block:: bash

      
      git clone https://github.com/RainerKuemmerle/g2o.git
      mkdir build
      cd build
      make -j6
      sudo make install


sophus

.. code-block:: bash

      
      git clone https://github.com/strasdat/Sophus.git
      mkdir build
      cd build
      make -j6
      sudo make install

Build
=====

We are now ready to build, test, and begin run the demos.

.. code-block:: bash
 
 mkdir build
 cd build
 cmake ..
 make -j6
 

Docker安装
============

  正在制作中，请耐心等待。。。