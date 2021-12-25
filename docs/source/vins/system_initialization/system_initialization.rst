.. highlight:: c++

.. default-domain:: cpp

=====================
System Initialization
=====================


相机初始化
========== 

.. figure:: ../images/system_initialize.png
    :align: center


相机坐标系 :math:`c_0` 为世界坐标系:

本体坐标系转换到 :math:`c_0` 坐标系:

.. math::

    \begin{align}
        q_{b_k}^{c_0} &= q_{c_k}^{c_0}\otimes (q_{c}^{b})^{-1} \\
        sp_{b_k}^{c_0} &= s\bar{p}_{c_k}^{c_0} - R_{b_k}^{c_0}p_{c}^{b} \\
    \end{align}

其中参数 :math:`s` 给视觉测量的位移赋予尺度信息。