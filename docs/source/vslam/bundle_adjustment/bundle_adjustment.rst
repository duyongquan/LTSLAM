.. highlight:: c++

.. default-domain:: cpp

=================
Bundle Adjustment
=================
本文翻译于 `Bundle Adjustment Revisited <https://arxiv.org/pdf/1912.03858.pdf>`_


I. Introduction
===============================

集束平差法在大地测量学和 3D 重建（SLAM 和 SfM）中发挥着重要作用，但直到最近才被认为已解决。集束束调整构成大多数最先进的多视图几何系统中的核心组件，
并且通常作为最终细化阶段调用，以近似初始场景估计以及在增量重建中消除漂移的方法。 Levenberg-Marquardt 算法已被证明是解决该公式的最成功的方法，、
因为它易于实现、对初始化具有鲁棒性，并且其框架非常适合利用现在的稀疏形式的多视图几何的问题上。该算法的每一步都会产生一个参数估计值，
该估计值在前一个的基础上有所改进，由此产生的一系列迭代可以显示为收敛到手头目标函数的局部最小值。

因此，在过去十年中提出了许多捆绑调整的方法。 这些方法分为两组：第一个分支侧重于使捆绑调整算法尽可能高效，
而第二个分支侧重于减少单个捆绑调整的大小或调用频率。

集束调整是细化视觉重建以产生联合最优 3D 结构和观看参数估计的关键。 最优意味着通过最小化一些量化模型拟合误差的成本函数来找到参数估计，
同时，解决方案在结构和相机变化方面同时是最优的。 该名称指的是离开每个 3D 特征并会聚在每个相机中心的光线“束”，
这些光线针对特征和相机位置进行了最佳“调整”。

本文的主要目的有两个：

* 1 从理论和实践层面对集束平差法调整问题进行详细探索和推导。
* 2 展示平行束平差的发展历程，并给出未来束平差的方向。

我们的论文组织如下：首先在第二部分介绍投影相机模型和相机失真，然后展示如何使用引入的相机模型进行束调整。 
在第三节中，我们详细给出了传统捆绑调整的推导，分布式捆绑调整算法在第四节中给出。 最后，对我们的工作做一个总结。

II. Projection Camera Model
===============================

束调整描述了测量的像素坐标 :math:`u_{ij}` 和重新投影的像素坐标之间的误差总和。重新投影的像素坐标由结构（世界框架中的 3D 点坐标）
和相机参数计算得出。 因此，在我们深入研究捆绑调整之前，必须弄清楚重新投影的过程。 我们将在本节首先介绍针孔相机模型。

A. pinhole camera
-------------------------------

针孔相机模型如图1所示，相机坐标系 :math:`o-xyz` 中 :math:`p` 坐标为 :math:`(X,Y,Z)^T` ，相机焦距为 :math:`f` ，相机中心 :math:`o` 为针孔， 
并且 :math:`p` 被投影到成像平面 :math:`o-xy` 中，并由二维点 :math:`p^{\prime}` 表示。

.. figure:: ./images/camera_pinhole_model.png
    :align: center

    Figure 1. Pinhole camera model: p is a 3D point located in camera frame, o is
    the camera center, f is the focal length, p is projected into the imaging plane
    on a pinhole camera model.


.. figure:: ./images/similar_triangle.png
   :align: center

   Figure 2. Similar Triangle

基于相似三角形理论（如图 2 所示），我们可以获得：

.. math:: 

    \frac{f}{Z} = -\frac{X^{\prime}}{X} = -\frac{Y^{\prime}}{Y} \tag{1}

然后（1）简化为 :math:`\frac{f}{Z} = \frac{X^{\prime}}{X} = \frac{Y^{\prime}}{Y}`, 那么

.. math::

  \begin{align}
    \begin{cases}
      X^{\prime} = f \frac{X}{Z} \\
      Y^{\prime} = f \frac{Y}{Z}
    \end{cases}  
  \end{align}  


为了简化模型，我们可以将成像平面连同 3D 点放在相机前面，如图 3 所示：

.. figure:: ./images/imaging_plane.png
   :align: center

   Figure 3. Imaging Plane: (a) is the real imaging plane, (b) is the symmetry
   imaging plane that we put the imaging plane in (a) in front of camera.

但是，我们只能得到成像平面上的坐标，而实际上我们得到的是像素平面上的像素坐标。 假设在成像平面 :math:`ouv` 上固定有一个像素平面，
点 :math:`p^{\prime}` 在像素平面上的坐标为 :math:`(µ, ν)^T` 。 与比例和平移相关的像素帧和成像平面。 假设像素坐标在 :math:`u` 轴上按 :math:`α` 缩放，
在 :math:`v` 轴按 :math:`β` 缩放，到原点的平移是 :math:`(c_x , c_y )^T` 。 则成像平面坐标与像素平面坐标的关系为：

.. math::

  \begin{align}
    \begin{cases}
      \mu = \alpha X^{\prime}  + c_x\\
      \nu = \beta Y^{\prime}  + c_y
    \end{cases}  
  \end{align}  

将(2) 代入(3) 并设 :math:`f_x = \alpha · f` , :math:`f_y = \beta · f` ，我们可以得到：

.. math::

  \begin{align}
    \begin{cases}
      \mu = f_x \frac{X}{Z}  + c_x \\
      \nu = f_y \frac{Y}{Z}  + c_y 
    \end{cases}  
  \end{align}  

通常，(4) 写成矩阵形式：

.. math::

    \begin{bmatrix}
        \mu \\
        \nu \\
        1
    \end{bmatrix} =
    \begin{bmatrix}
        f_x & 0   & c_x  \\
        0    & f_y & c_y  \\
        0    & 0   & 1  
    \end{bmatrix} =
      \begin{bmatrix}
        \frac{X}{Z} \\
        \frac{Y}{Z} \\
        1
    \end{bmatrix} =
    \frac{1}{Z} KP

其中 :math:`K` 称为校准矩阵或内部矩阵。

上式中，:math:`P` 的 3D 坐标由于相机运动而位于相机坐标系中，而实际上我们只能获得其在世界坐标系 :math:`P_w` 中的坐标。 
所以我们必须先将 :math:`P_w` 转换为相机帧，然后才能使用它：

.. math::

    p_{\mu \nu} = 
    \begin{bmatrix}
        \mu \\
        \nu \\
        1
    \end{bmatrix} =
    \frac{1}{Z} K(RP_w + t)

其中 :math:`P_c = \frac{1}{Z}(RP_w + t)` 称为归一化坐标。 它位于 :math:`Z = 1` 的相机平面的前面，该平面称为图4中的归一化平面

.. figure:: ./images/normalized_imaging_plane.png
   :align: center

   Figure 4. Normalized Imaging Plane

B. camera distortion
-------------------------------

为了获得更好的成像效果，通常在摄像头前加装光学镜头，这样在成像过程中可能会影响到光线的预测。 主要有两个原因：

* 受光学透镜影响的光线通过路径
* 组装相机时光学镜头与成像平面不严格平行

**径向畸变**

由光学透镜本身引起的畸变称为径向畸变。 相机镜头使图像中的直线变成曲线，越靠近图像的边界越清晰。 径向畸变通常分为桶形畸变和枕形畸变。
对于径向畸变，我们可以使用与图像中心距离相关的多项式函数对其进行校正。 径向畸变校正方程由下式给出

.. math::

    \begin{align}
      \begin{cases}
        x_c = x(1 + k_1r^2 + k_2r^4 + k_3r^6) \\
        x_y = y(1 + k_1r^2 + k_2r^4 + k_3r^6) 
      \end{cases}  
    \end{align}  

其中 :math:`(x, y)^T` 为校正前坐标，:math:`(x_c , y_c )^T` 为校正后坐标，:math:`r = x^2 + y^2` 。 
请注意，:math:`(x, y)^T` 和 :math:`(x_c , y_c )^T` 都位于归一化图像平面中


**切向畸变**

如本节开头所述，切向畸变是由光学镜头与像平面之间不严格平行造成的。 在切向畸变中，我们可以使用另外两个参数 :math:`p_1 , p_2` 进行校正。 
切向畸变校正方程由下式给出

.. math::

    \begin{align}
      \begin{cases}
        x_c = x + 2p_1xy + p_2(r^2 + 2x^2)  \\
        y_c = y + p_1(r^2 + 2y^2) + 2p_2xy 
      \end{cases}  
    \end{align}  

**使用相机模型执行捆绑调整**

有了上面的讨论，我们可以对重投影过程做一个总结，由算法1给出

.. figure:: ./images/Structure_Re-projection_Algorithm.png
   :align: center

   Algorithm 1. Structure Re-projection

III. Conventional Bundle Adjustment
===================================


.. figure:: ./images/Bundle_Adjustment.png
   :align: center

   Figure 5. Bundle Adjustment: :math:`u_{ij}` is the observations, :math:`X_i` is the 3D points
   in world frame, :math:`C_j` is the camera center, :math:`u_{ij}` is the re-projected 2D points.
   The solid line represents the projection procedure, the dotted line represents
   the re-projection procedure.

束调整常用于从运动到结构的结构和 SLAM 的后端。 它试图最小化 2D 观测值和预测的 2D 点之间的误差总和，其中预测点通过相机参数从 3D 结构重新投影。 
它测量计算的 3D 结构和相机参数的准确性。 如图 5 所示，束调整实际上是一个非线性最小二乘问题，由以下等式描述

.. math::

    min \sum_{i=1}^{n} \sum_{j=1}^{m}(u_{ij} - \pi(C_j, X_i))^2   \tag{11}

其中 :math:`u_{ij}` 是像素级的观察点坐标，表示第 :math:`j` 个摄像机 :math:`C_j` 观察到的第 :math:`i` 个 3D 点 :math:`X_i` 。 :math:`\pi(C_j , X_i)` 
是第三节中描述的非线性运算

为了简化符号，令 :math:`r_{ij} = u_{ij} − \pi(C_j , X_i)` ，然后我们可以将 (11) 重写为

.. math::

    \min \mathbf{r}^T\mathbf{r} \tag{12}

然后，我们执行一阶泰勒展开

.. math::

    r(x + \delta x) = r(x) + g^T \delta x + \frac{1}{2} \delta x^T H \delta x \tag{13}

其中 :math:`g` 和 :math:`H` 分别是 :math:`r` 的梯度和 :math:`Hessian`。 通过对方程 13 求导并将其设置为零，我们得到

.. math::

    H \delta x = -g \tag{14}

对于最小二乘问题，:math:`H = J^TJ + S，g = J^Tr` ，其中 :math:`S = \sum_{i=1}^{n} \sum_{j=1}^{m} r_{ij} \nabla^{2} r_{ij}` 。
如果 :math:`S` 足够小，那么通过省略 :math:`S` ，我们得到 Gauss-Newton 方程

.. math::

  J^TJ \delta x = -J^Tr \tag{15}

注意，如果 :math:`J^TJ` 是正定的，Gauss-Newton 方法总是可以产生一个减小的方向，
因为 :math:`(\delta x)^Tg = -(\delta x)^T J^Tr = -(\delta x)^T J^T J\delta x` < 0。
但是当 :math:`J^TJ` 变得奇异时，则 Gauss-Newton 方法在数值上变得不稳定。 为了克服 Gauss-Newton 的弱点，
我们可以改用 Levenberg-Marquardt 方法。

.. math::

  (J^TJ + \lambda I) \delta x= -J^Tr \tag{16}


A. reduced camera system
--------------------------------------------

B. bundle adjustment with conjugate gradient
--------------------------------------------

IV. Distributed Bundle Adjustment
============================================

V. Conclusion
============================================

在本文中，我们以传统方式和分布式方式对捆绑调整问题进行了详细的推导和各种解决方案。 
很明显，预处理共轭梯度方法可以替代基于稠密 Cholesky 分解的方法。 
虽然分布式束调整方法易于解决非常大规模的重建问题，但它们在准确性和效率方面都超过了从小到大规模问题的传统方法。

VI. Reference
============================================

1 `Bundle Adjustment Revisited <https://arxiv.org/pdf/1912.03858.pdf>`_
