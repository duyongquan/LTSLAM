.. highlight:: c++

.. default-domain:: cpp

=======================
Ch07 Kalman Filter Math
=======================


Modeling a Dynamic System
=========================

动态系统模型，例如，一辆以一定速度行驶的汽车在固定的时间内行驶这么远，它的速度随着它的加速度而变化。
我们用我们在高中学到的众所周知的牛顿方程来描述这种行为。

.. math::

    \begin{aligned}
    v&=at\\
    x &= \frac{1}{2}at^2 + v_0t + x_0
    \end{aligned}


根据微积分数学知识，我们会得到：

.. math::

    \mathbf v = \frac{d \mathbf x}{d t}, 
    \quad \mathbf a = \frac{d \mathbf v}{d t} = \frac{d^2 \mathbf x}{d t^2}

系统建模简化。 在任何时间 $t$ 我们都说真实状态（例如我们汽车的位置）是来自不完美模型的预测值加上一些未知的*过程噪声*：

.. math::

    x(t) = x_{pred}(t) + noise(t)

将一组高阶微分方程转换为一组一阶微分方程。转换后的无噪声系统模型为：

.. math::

    \dot{\mathbf x} = \mathbf{Ax}

:math:`\mathbf A` 被称为*系统动力学矩阵*。对噪声进行建模，称其为 :math:`\mathbf w`，并将其添加到等式中

.. math::

    \dot{\mathbf x} = \mathbf{Ax} + \mathbf w


:math:`\mathbf w` 可能会让你觉得这个名字是一个糟糕的选择，但你很快就会看到卡尔曼滤波器假设*白*噪声。


State-Space Representation of Dynamic Systems
=============================================


Forming First Order Equations from Higher Order Equations
==========================================================

First Order Differential Equations In State-Space Form
======================================================

Finding the Fundamental Matrix for Time Invariant Systems
==========================================================

The Matrix Exponential
======================

Time Invariance
=============== 

Example: Mass-Spring-Damper Model
=================================

Linear Time Invariant Theory
============================


Numerical Solutions
=================== 

Design of the Process Noise Matrix
==================================

Continuous White Noise Model
============================ 


Piecewise White Noise Model
===========================


Stable Compution of the Posterior Covariance
============================================


Deriving the Kalman Gain Equation
=================================

Numeric Integration of Differential Equations
=============================================

Euler's Method
==============

Runge Kutta Methods
===================

Bayesian Filtering
==================

Converting Kalman Filter to a g-h Filter
========================================


