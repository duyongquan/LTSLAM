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

我们已经推导出方程

.. math::
    
    \dot{\mathbf x} = \mathbf{Ax}+ \mathbf{Bu} + \mathbf{w}

然而，我们对 :math:`\mathbf x` 的导数不感兴趣，而对 :math:`\mathbf x` 本身感兴趣。 暂时忽略噪音，
我们需要一个方程，根据时间 :math:`t_{k-1}` 处的 :math:`\mathbf x` 递归地求出时间 :math:`t_k` 处的 :math:`\mathbf x` 值：

.. math::
    
    \mathbf x(t_k) = \mathbf F(\Delta t)\mathbf x(t_{k-1}) + \mathbf B(t_k)\mathbf u (t_k)

约定允许我们将 :math:`\mathbf x(t_k)` 写成 :math:`\mathbf x_k`，这意味着

:math:`\mathbf x` 在 :math:`t` 的 k :math:`^{th}` 值处的值。

.. math::
    
    \mathbf x_k = \mathbf{Fx}_{k-1} + \mathbf B_k\mathbf u_k

:math:`\mathbf F` 是熟悉的*状态转换矩阵*，因其能够在离散时间步之间转换状态值而得名。
它与系统动力学矩阵 :math:`\mathbf A` 非常相似。 不同之处在于 :math:`\mathbf A` 对一组线性微分方程进行建模，
并且是连续的。 :math:`\mathbf F` 是离散的，表示一组线性方程（不是微分方程），它在离散时间步 :math:`\Delta t` 
上将 :math:`\mathbf x_{k-1}` 转换为 :math:`\mathbf x_k` 。

找到这个矩阵通常非常困难。 方程 :math:`\dot x = v` 是最简单的微分方程，我们将它简单地积分为：

.. math::
    
    \int\limits_{x_{k-1}}^{x_k}  \mathrm{d}x = \int\limits_{0}^{\Delta t} v\, \mathrm{d}t 

.. math::
    
    x_k-x_0 = v \Delta t

.. math::
    
    x_k = v \Delta t + x_0

这个方程是 *递归的* ：我们根据它在时间 :math:`t-1` 的值计算 :math:`x` 在时间 :math:`t` 的值。 
这种递归形式使我们能够以卡尔曼滤波器所需的形式表示系统（过程模型）：

.. math::
    
    \begin{aligned}
        \mathbf x_k &= \mathbf{Fx}_{k-1}  \\
        &= \begin{bmatrix} 1 & \Delta t \\ 0 & 1\end{bmatrix}
        \begin{bmatrix}x_{k-1} \\ \dot x_{k-1}\end{bmatrix}
    \end{aligned}

我们可以这样做只是因为 :math:`\dot x = v` 是可能的最简单微分方程。 几乎所有其他物理系统都导致更复杂的微分方程不屈服于这种方法。

*状态空间*方法在阿波罗任务期间开始流行，主要是由于卡尔曼博士的工作。 这个想法很简单。 
用一组 :math:`n^{th}` 阶微分方程对系统建模。 将它们转换为一组等效的一阶微分方程。 
将它们放入上一节中使用的向量矩阵形式：:math:`\dot{\mathbf x} = \mathbf{Ax} + \mathbf{Bu}` 。 
一旦采用这种形式，我们使用几种技术将这些线性微分方程转换为递归方程：

.. math::
    
    \mathbf x_k = \mathbf{Fx}_{k-1} + \mathbf B_k\mathbf u_k

有些书将状态转移矩阵称为*基本矩阵*。 许多人使用 :math:`\mathbf \Phi` 而不是 :math:`\mathbf F`。 
大量基于控制理论的资料倾向于使用这些形式。这些被称为*状态空间*方法，因为我们用系统状态来表达微分方程的解。

Forming First Order Equations from Higher Order Equations
==========================================================

许多物理系统模型需要具有控制输入 :math:`u` 的二阶或更高阶微分方程：

.. math::

    a_n \frac{d^ny}{dt^n} + a_{n-1} \frac{d^{n-1}y}{dt^{n-1}} +  \dots + a_2 \frac{d^2y}{dt^2} + a_1 \frac{dy}{dt} + a_0 = u

高阶方程组简化为一阶, 给定系统 :math:`\ddot{x} - 6\dot x + 9x = u` 找到等效的一阶方程

第一步是将最高阶项隔离到等式的一侧。

.. math::

    \ddot{x} = 6\dot x - 9x + u

我们定义了两个新变量：

.. math::

    \begin{aligned} x_1(u) &= x \\
    x_2(u) &= \dot x
    \end{aligned}

现在我们将这些代入原始方程并求解。 该解根据这些新变量产生一组一阶方程。 为方便起见，通常会删除 :math:`(u)` 。

我们知道 :math:`\dot x_1 = x_2 和 \dot x_2 = \ddot{x}` 。 所以

.. math::

    \begin{aligned}
    \dot x_2 &= \ddot{x} \\
            &= 6\dot x - 9x + t\\
            &= 6x_2-9x_1 + t
    \end{aligned}


因此我们的一阶方程组是

.. math::

    \begin{aligned}
        \dot x_1 &= x_2 \\
        \dot x_2 &= 6x_2-9x_1 + t
    \end{aligned}

如果你稍微练习一下，你就会熟练掌握它。 隔离最高项，定义一个新变量及其导数，然后替换。


First Order Differential Equations In State-Space Form
======================================================

替换上一节中新定义的变量：

.. math::

    \frac{dx_1}{dt} = x_2,\,  
    \frac{dx_2}{dt} = x_3, \, ..., \, 
    \frac{dx_{n-1}}{dt} = x_n

一阶方程得到：

.. math::
    
    \frac{dx_n}{dt} = \frac{1}{a_n}\sum\limits_{i=0}^{n-1}a_ix_{i+1} + \frac{1}{a_n}u

使用向量矩阵表示法，我们有：

.. math::

    \begin{bmatrix}\frac{dx_1}{dt} \\ \frac{dx_2}{dt} \\ \vdots \\ \frac{dx_n}{dt}\end{bmatrix} = 
    \begin{bmatrix}\dot x_1 \\ \dot x_2 \\ \vdots \\ \dot x_n\end{bmatrix}=
    \begin{bmatrix}0 & 1 & 0 &\cdots & 0 \\
    0 & 0 & 1 & \cdots & 0 \\
    \vdots & \vdots & \vdots & \ddots & \vdots \\
    -\frac{a_0}{a_n} & -\frac{a_1}{a_n} & -\frac{a_2}{a_n} & \cdots & -\frac{a_{n-1}}{a_n}\end{bmatrix}
    \begin{bmatrix}x_1 \\ x_2 \\ \vdots \\ x_n\end{bmatrix} + 
    \begin{bmatrix}0 \\ 0 \\ \vdots \\ \frac{1}{a_n}\end{bmatrix}u

然后我们写成 :math:`\dot{\mathbf x} = \mathbf{Ax} + \mathbf{B}u` .

Finding the Fundamental Matrix for Time Invariant Systems
==========================================================

我们用状态空间形式表示系统方程

.. math::
    
    \dot{\mathbf x} = \mathbf{Ax}

其中 :math:`\mathbf A`` 是系统动力学矩阵，并且想要找到*基本矩阵* :math:`\mathbf F`
在区间 :math:`\Delta t` 上传播状态 :math:`\mathbf x` 与方程

.. math::
    
    \begin{aligned}
    \mathbf x(t_k) = \mathbf F(\Delta t)\mathbf x(t_{k-1})\end{aligned}
    
换句话说，:math:`\mathbf A` 是一组连续微分方程，我们需要 :math:`\mathbf F`` 是一组离散线性方程组，
用于计算在离散时间步长上 :math:`\mathbf A` 的变化。

按照惯例，去掉 :math:`t_k` 和 :math:`(\Delta t)` 并使用符号

.. math::
    
    \mathbf x_k = \mathbf {Fx}_{k-1}

一般来说，有三种常见的方法可以找到卡尔曼滤波器的这个矩阵。 最常用的技术是矩阵指数。 
线性时不变理论，也称为 LTI 系统理论，是第二种技术。 最后，还有数值技术。 您可能知道其他人，
但这三个是您在卡尔曼滤波器文献和实践中最有可能遇到的。

The Matrix Exponential
======================

方程 :math:`\frac{dx}{dt} = kx` 的解可以通过以下方式找到：

.. math::
    
    \begin{gathered}
        \frac{dx}{dt} = kx \\
        \frac{dx}{x} = k\, dt \\
        \int \frac{1}{x}\, dx = \int k\, dt \\
        \log x = kt + c \\
        x = e^{kt+c} \\
        x = e^ce^{kt} \\
        x = c_0e^{kt}
    \end{gathered}

使用类似的数学，一阶方程的解

.. math::
    
    \dot{\mathbf x} = \mathbf{Ax} ,\, \, \, \mathbf x(0) = \mathbf x_0

where :math:`\mathbf A` is a constant matrix, is

.. math::
    
    \mathbf x = e^{\mathbf At}\mathbf x_0

代入 :math:`F = e^{\mathbf At}`，我们可以写成

.. math::
    
    \mathbf x_k = \mathbf F\mathbf x_{k-1}

这是我们正在寻找的形式！ 我们将求基本矩阵的问题简化为求 :math:`e^{\mathbf At}` 的值。

:math:`e^{\mathbf At}` 被称为[矩阵指数](https://en.wikipedia.org/wiki/Matrix_exponential)。 可以用这个幂级数计算：

.. math::
    
    e^{\mathbf At} = \mathbf{I} + \mathbf{A}t  + \frac{(\mathbf{A}t)^2}{2!} + \frac{(\mathbf{A}t)^3}{3!} + ...

该级数是通过对 :math:`e^{\mathbf At}` 进行泰勒级数展开得到的，这里我不会介绍。

让我们用它来找到牛顿方程的解。 使用 :math:`v` 代替 :math:`\dot x`，并假设速度恒定，我们得到线性矩阵向量形式

.. math::
    
    \begin{bmatrix}
        \dot x \\ 
        \dot v\end{bmatrix} =\begin{bmatrix}0&1\\
        0&0\end{bmatrix} \begin{bmatrix}x \\ v
    \end{bmatrix}


这是一个一阶微分方程，因此我们可以设置 :math:`\mathbf{A}=\begin{bmatrix}0&1\\0&0\end{bmatrix}` 并求解以下方程。
我已将区间 :math:`\Delta t`` 替换为 :math:`t` 以强调基本矩阵是离散的：

.. math::
    
    \mathbf F = e^{\mathbf A\Delta t} = \mathbf{I} + \mathbf A\Delta t  + \frac{(\mathbf A\Delta t)^2}{2!} + \frac{(\mathbf A\Delta t)^3}{3!} + ...

如果你执行乘法你会发现:math:`\mathbf{A}^2=\begin{bmatrix}0&0\\0&0\end{bmatrix}`，这意味着:math:`\mathbf{A}`
的所有高次幂也是 :math:`\mathbf{0}`. 因此，我们得到了一个没有无数项的准确答案：

.. math::

    \begin{aligned}
        \mathbf F &=\mathbf{I} + \mathbf A \Delta t + \mathbf{0} \\
        &= \begin{bmatrix}1&0\\0&1\end{bmatrix} + \begin{bmatrix}0&1\\0&0\end{bmatrix}\Delta t\\
        &= \begin{bmatrix}1&\Delta t\\0&1\end{bmatrix}
    \end{aligned}

我们把它代入 :math:`\mathbf x_k= \mathbf{Fx}_{k-1}` 得到

.. math::

    \begin{aligned}
        x_k &=\begin{bmatrix}1&\Delta t\\0&1\end{bmatrix}x_{k-1}
    \end{aligned}

您将认识到这是我们在**多变量卡尔曼滤波器**一章中为恒速卡尔曼滤波器分析得出的矩阵。

SciPy 的 linalg 模块包括一个例程“expm()”来计算矩阵指数。它不使用泰勒级数方法，
而是使用 [Padé Approximation](https://en.wikipedia.org/wiki/Pad%C3%A9_approximant)。
计算矩阵指数的方法有很多（至少 19 种），并且都存在数值困难[1]。您应该意识到这些问题，尤其是当 :math:`\mathbf A` 很大时。
如果您搜索“pade 逼近矩阵指数”，您会发现许多专门针对此问题的出版物。

在实践中，您可能并不关心卡尔曼滤波器，我们通常只取泰勒级数的前两项。但是不要假设我对这个问题的处理是完整
的并且跑掉并尝试将这种技术用于其他问题而不对这种技术的性能进行数值分析。有趣的是，求解 :math:`e^{\mathbf At}` 
的一种受欢迎的方法是使用广义 ode 求解器。换句话说，他们的做法与我们的做法相反——将 :math:`\mathbf A` 转化为一组微分方程，
然后使用数值技术求解该组！

这是一个使用 `expm()` 求解 :math:`e^{\mathbf At}` 的示例。


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


