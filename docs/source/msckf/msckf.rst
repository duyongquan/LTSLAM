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

2 符号表示
================

.. figure:: ./images/coordinate_frames.png
   :align: center

* G : 世界坐标系
* C : 相机坐标系
* B : 机体坐标系


3 概率状态估计
================

**概率论基础**

:math:`P(x = X)` 在某个范围内的概率等于 概率密度函数 :math:`p(x)` 在该范围内的积分

.. figure:: ./images/probability_density_function.png
   :align: center

均值和方差

* :math:`\mathbb{E}(x) = \int x p(x) dx`   
* :math:`Var(x) = \mathbb{E}[x - \mathbb{E}(x)^2] = \sigma^2`

**高斯分布**

一维高斯密度函数

.. math:: 

  p(x; \mu, \sigma) = \frac{1}{\sqrt{2 \pi \sigma^2}} exp
  \left\{ 
    -\frac{(x- \mu)^2}{2\sigma^2}
  \right\}

.. figure:: ./images/normal_distribution.png
   :align: center


:math:`N` 维高斯密度函数

.. math:: 

  p(\mathbf{x}; \mathbf{\mu}, \mathbf{\Sigma}) =
  \frac{1}{\sqrt{(2\pi)^2|\mathbf{\Sigma}|}} exp
  \left\{ 
    -\frac{1}{2} (\mathbf{x}- \mathbf{\mu})^T\Sigma^{-1}(\mathbf{x}- \mathbf{\mu})
  \right\}


其中：

  .. math:: 

    \begin{aligned}
    Cov(\mathbf{x}, \mathbf{y}) &= \mathbb{E[(\mathbf{x} - \mathbb{E(x)})(\mathbf{y} - \mathbb{E(y)})]} \\
    Cov\left(\begin{bmatrix}
      x_1 \\
      x_2 \\
      \vdots \\
      x_n
    \end{bmatrix}
    \right) &=
    \begin{bmatrix}
      \sigma_{x_1}^2 & \rho_{(x_1, x_2)}\sigma_{x_1}\sigma_{x_2} & \dots & \rho_{(x_1, x_n)}\sigma_{x_1}\sigma_{x_n}  \\
      \rho_{(x_2, x_1)}\sigma_{x_2}\sigma_{x_1} & \sigma_{x_2}^2 & \dots & \rho_{(x_2, x_n)}\sigma_{x_2}\sigma_{x_n}  \\
      \vdots & \vdots & \ddots & \vdots  \\
      \rho_{(x_n, x_1)}\sigma_{x_n}\sigma_{x_1} &  \rho_{(x_n, x_2)}\sigma_{x_n}\sigma_{x_2} & \dots  & \sigma_{x_n}^2
    \end{bmatrix}
    \end{aligned} 

.. figure:: ./images/two-dimensional_normal_distribution.png
   :align: center

**条件高斯**

.. math:: 

    \begin{bmatrix}
      \mathbf{x} \\
      \mathbf{y} 
    \end{bmatrix} =
    N(\mathbf{\mu}, \mathbf{\Sigma}) =
    \left(
        \begin{bmatrix}
            \mu_x \\
            \mu_y 
        \end{bmatrix},
        \begin{bmatrix}
            \Sigma_{xx} & \Sigma_{xy} \\
            \Sigma_{yx} & \Sigma_{yy} 
        \end{bmatrix}
    \right)

边缘化

.. math:: 

  p(x) = \int p(x,y) dy = \int p(x|y) p(y)dy = N(\mu_x, \Sigma_{xx})

条件概率

.. math:: 

    p\left(
    \begin{bmatrix}
        x \\
        y 
    \end{bmatrix}
    \right) 
    =
    N\left(
        \begin{bmatrix}
            \mu_x \\
            \mu_y 
        \end{bmatrix},
        \begin{bmatrix}
            \Sigma_{xx} & \Sigma_{xy} \\
            \Sigma_{yx} & \Sigma_{yy} 
        \end{bmatrix}
    \right)
    =
    N\left(
        \begin{bmatrix}
            \mu_x \\
            A\mu_x + b 
        \end{bmatrix},
        \begin{bmatrix}
            \Sigma_{xx} & \Sigma_{xx}A^T \\
            A\Sigma_{xx} & A\Sigma_{xx}A^T + Q 
        \end{bmatrix}
    \right)

其中：

* :math:`\mathbf{x} \sim N(\mu_x, \Sigma_{xx})`
* :math:`y = Ax + b, \quad b \sim N(0, Q)`

4 卡尔曼滤波
================

**卡尔曼滤波**

初始状态估计

.. math:: 

    \mathbf{x_0} \sim N(\hat{\mathbf{x}}_{0|0}, \mathbf{\Sigma}_{0|0})

预测

.. math::

    \mathbf{Given}: \mathbf{x}_{t+1} = \mathbf{A}_{t}\mathbf{x}_{t} + \mathbf{B}_{t}\mathbf{u}_{t} + \mathbf{\epsilon}_{t} \quad \mathbf{\epsilon}_{t} \sim N(\mathbf{0}, \mathbf{Q}_t)
    \\
    \begin{aligned}
      \hat{\mathbf{x}}_{t+1|t} &= \mathbf{A}_{t|t}\hat{\mathbf{x}}_{t} + \mathbf{B}_{t}\mathbf{u}_{t} \\
      \hat{\mathbf{\Sigma}}_{t+1|t} &= \mathbf{A}_{t}\hat{\mathbf{\Sigma}}_{t}\mathbf{A}_{t}^T + \mathbf{Q}_{t}
    \end{aligned}

更新

.. math::

  \mathbf{Given}: \mathbf{x}_{t} = \mathbf{C}_{t}\mathbf{x}_{t} + 	\mathbf{\epsilon}_{t} \quad \mathbf{\delta}_{t} \sim N(\mathbf{0}, \mathbf{R}_t)
  \\
  \begin{aligned}
    \hat{\mathbf{x}}_{t|t} &= \hat{\mathbf{x}}_{t|t-1} + \mathbf{K}_{t}(\mathbf{z}_t - C_t\hat{\mathbf{x}}_{t|t-1}) \\
    \mathbf{\Sigma}_{t|t} &= \mathbf{\Sigma}_{t|t-1} - K_t C_t \mathbf{\Sigma}_{t|t-1} \\
    \mathbf{K}_{t} &= \mathbf{\Sigma}_{t|t-1}C_t^T(C_t\mathbf{\Sigma}_{t|t-1}C_t^T + R_t)^{-1}
  \end{aligned}

5 IMU
================

**Accelerometers(加速计)**

.. math::

    ^B \mathbf{a}_m = \mathbf{T}_a {_G^B}\mathbf{R}(^G\mathbf{a} - ^G\mathbf{g}) + \mathbf{n}_a + \mathbf{b}_a

其中：

* :math:`\mathbf{T}_a` : 加速度计测量中导致未对准和比例误差的矩阵系数
* :math:`^G\mathbf{a}` : 全局坐标系中 IMU 的真实加速度，{ B } 表示惯性体（IMU）坐标系。
* :math:`^G\mathbf{g}: \quad \mathbf{g} = (0, 0, -1)^T`  

* :math:`\mathbf{n}_a \sim N(0, N_a)` 
* :math:`\mathbf{b}_a：` 随时间变化，建模为随机游走过程噪声 :math:`n_{wa} \sim N(0,N_{wa} )` 

**Gyroscope(陀螺仪)**

.. figure:: ./images/gyroscope.png
   :align: center

.. math::

    ^B \mathbf{\omega}_m = \mathbf{T}_g \omega +\mathbf{T}_s ^B\mathbf{a} + \mathbf{n}_g + \mathbf{b}_g

其中：

* :math:`\mathbf{n}_g \sim N(0, N_g)` 
* :math:`\mathbf{b}_g：` 随时间变化，建模为随机游走过程噪声 :math:`n_{wg} \sim N(0,N_{wg})` 


6 Computer Vision
=================


7 MSCKF-VIO
================


