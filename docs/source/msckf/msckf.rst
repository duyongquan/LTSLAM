.. _chapter-msckf:

=====
MSCKF
=====

`代码: MSCKF_VIO <https://github.com/KumarRobotics/msckf_vio>`_ 

`论文: Robust Stereo Visual Inertial Odometry for Fast Autonomous Flight <https://arxiv.org/pdf/1712.00036.pdf>`_

1 介绍
=====================

**编译安装**

.. code-block:: bash

  # 安装依赖
  sudo apt-get install libsuitesparse-dev

  cd your_work_space
  catkin_make --pkg msckf_vio --cmake-args -DCMAKE_BUILD_TYPE=Release


**运行**

* 下载  EuRoC or the UPenn fast flight dataset

.. code-block:: bash

  # EuRoC
  roslaunch msckf_vio msckf_vio_euroc.launch

  # UPenn fast flight 
  roslaunch msckf_vio msckf_vio_fla.launch

  # rosbag
  rosbag play V1_01_easy.bag

  # RVIZ
  rosrun rviz rosrun


**MSCKF的ros graph**

.. image:: ./images/msckf_vio.png
   :align: center


**视频结果**

* `bilibili <https://www.bilibili.com/video/BV1hM4y1g7N9?spm_id_from=333.337.search-card.all.click>`_
* `youtube <https://www.youtube.com/watch?v=jxfJFgzmNSw&t>`_ 

2 Kalman Filter
================




