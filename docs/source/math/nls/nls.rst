.. highlight:: c++

.. default-domain:: cpp

=========================
Non-linear Least Squares
=========================

Introduction and definitions
-----------------------------

.. NOTE::

    Definition Least Squares Problem

    .. math::

        F(X) = \frac{1}{2} \sum_{i=1}^{m}(f_i(X))^{2}

    where :math:`f_i:R^{n} \in R, i = 1, \cdots` , 
    :math:`m` are given functions, and :math:`m \leq n`;
        

.. figure :: ../images/nls/nls_1.png
    :align: center

fitting model:

.. math:: M(\mathbf{x}, t) = x_3 e^{x_1 t} + x_4 e^{x_2 t}


The model depends on the parameters :math:`x = [x_1; x_2; x_3; x_4]^{T}` . We assume that
there exists an :math:`x^{\star}` so that

.. math:: y_i = M(\mathbf{x}^{\star}, t_i) + \epsilon_{i}

where :math:`\epsilon_{i}` are (measurement) errors on the data ordinates, assumed to behave like “white noise”.

For any choice of x we can compute the residuals

.. math:: 

    \begin{align}
        f_i(\mathbf{x}) &= y_i - M(\mathbf{x}^{\star}, t_i) \\
                        &= y_i - x_3 e^{x_1 t} - x_4 e^{x_2 t}
    \end{align}

We assume that the cost function F is differentiable and so smooth that the
following Taylor expansion is valid

.. math::

    F(\mathbf{x} + \mathbf{h}) = F(\mathbf{x}) + \mathbf{h}^{T}\mathbf{g} + \frac{1}{2}\mathbf{h}^{T}H\mathbf{h} + O(||\mathbf{h}||^{3})

where :math:`g` is the gradient

.. math:: 

    \mathbf{g} =  F^{\prime}(\mathbf{x}) = 
    \begin{bmatrix}
        \frac{\partial{F}}{\partial{x}_1} \mathbf{x} \\
        \vdots \\
        \frac{\partial{F}}{\partial{x}_n} \mathbf{x}
    \end{bmatrix}

and :math:`H` is the Hessian

.. math:: 

    H =  F^{\prime \prime}(\mathbf{x}) = 
    \begin{bmatrix}
        \frac{\partial^{2}{F}}{\partial{x}_i \partial{x}_j} \mathbf{x}
    \end{bmatrix}

Descent Methods
-----------------

The Steepest Descent method
::::::::::::::::::::::::::::


Newton’s Method
:::::::::::::::

Line Search
:::::::::::


Trust Region and Damped Methods
:::::::::::::::::::::::::::::::

Non-linear Least Squares Problems
---------------------------------

The Gauss–Newton Method
::::::::::::::::::::::::

梯度gradient, 由多元函数的各个偏导数组成的向量。以二元函数为例，其梯度为：

.. math::

    \nabla{f(x_1, x_2)} = 
    \begin{bmatrix}
        \frac{\partial{f}}{\partial{x_1}}, \frac{\partial{f}}{\partial{x_2}}
    \end{bmatrix}

黑森矩阵Hessian matrix，由多元函数的二阶偏导数组成的方阵，描述函数的局部曲率，以二元函数为例

.. math::

    H({f(x_1, x_2)}) = 
    \begin{bmatrix}
        \frac{\partial{f}}{\partial{x_1^{2}}} & \frac{\partial{f}}{\partial{x_1 x_2}} \\
        \frac{\partial{f}}{\partial{x_1 x_2}} & \frac{\partial{f}}{\partial{x_2^{2}}}
    \end{bmatrix}

雅可比矩阵 Jacobian matrix，是多元函数一阶偏导数以一定方式排列成的矩阵，体现了一个可微方程与给出点的最优线性逼近。以二元函数为例

.. math::

    J(\boldsymbol{x}) = 
    \begin{bmatrix}
        \frac{\partial{f}}{\partial{x_1}} \\
        \frac{\partial{f}}{\partial{x_2}}
    \end{bmatrix}
    
如果扩展多维的话F: :math:`\mathbb{R}^n \mapsto \mathbb{R}^m` ，则雅可比矩阵是一个m行n列的矩阵

.. math:: 

    J(\boldsymbol{x}) = 
    \begin{bmatrix}
        \frac{\partial{f}_1}{\partial{x_1}} & \cdots & \frac{\partial{f}_1}{\partial{x_n}} \\
        \vdots & \ddots & \vdots \\
        \frac{\partial{f}_m}{\partial{x_1}} & \cdots & \frac{\partial{f}_m}{\partial{x_n}}
    \end{bmatrix}_{m \times n}

雅可比矩阵作用，如果P是 :math:`\mathbb{R}^n` 中的一点，F在P点可微分，那么在这一点的导数由 :math:`J_F(P)` 给出，在此情况下，
由 :math:`J_F(P)` 描述的线性算子即接近点P的F的最优线性逼近:

.. math:: F(x) \approx F(P) + J_F(P)(X - P)


牛顿法的基本思想是采用多项式函数来逼近给定的函数值，然后求出极小点的估计值，重复操作，直到达到一定精度为止。

.. NOTE:: 

    考虑如下一维无约束的极小化问题

    .. math::
        
        \min{f(x)} \quad x \in \mathbb{R}^{1} \\
        \phi(x) = f(x^{(k)}) + f^{\prime}(x^{(k)})(x - x^{(k)}) + \frac{1}{2}(x - x^{(k)})^2 \\
        \phi^{\prime}(x) = 0 \\
        则一维无约束极值的牛顿迭代公式为： 

        x^{(k+1)} = x^{(k)} - \frac{f^{\prime}(x^{(k)})}{f^{\prime \prime}(x^{(k)})}

    
需要注意的是，牛顿法在求极值的时候，如果初始点选取不好，则可能不收敛于极小点


试用牛顿法求 :math:`f(x) = \frac{3}{2}x_1^{2} + \frac{1}{2}x_2^{2} - x_1 x_2 - 2x_1` 的极小值

.. math::

    \nabla{f(x)} = 
    \begin{bmatrix}
        3x_1 - x_2 - 2 \\
        x_2 - x_1
    \end{bmatrix}

.. math:: 

    H(x) = \begin{bmatrix}
                3 & -1 \\
                -1 & 1
           \end{bmatrix}


取 :math:`x^{(0)} = (0, 0)^{T}`, 则

.. math::

    \nabla{f(x^{(0)})} = 
    \begin{bmatrix} 
        -2 \\
        0
    \end{bmatrix},
    H({f(x^{(0)})} = 
    \begin{bmatrix} 
        3  & -1 \\
        -1 &  1
    \end{bmatrix},
    H({f(x^{(0)})}^{-1} = 
    \begin{bmatrix} 
        \frac{1}{2}  & \frac{1}{2} \\
        \frac{1}{2}  & \frac{3}{2} 
    \end{bmatrix}

那么：

.. math::

    x^{(1)} = x^{(0)} - H({f(x^{(0)})}^{-1}\nabla{f(x^{(0)})} = 
    \begin{bmatrix} 
        0 \\
        0
    \end{bmatrix}
    \begin{bmatrix} 
        \frac{1}{2}  & \frac{1}{2} \\
        \frac{1}{2}  & \frac{3}{2} 
    \end{bmatrix}
    \begin{bmatrix} 
        -2 \\
        0
    \end{bmatrix}
    =
    \begin{bmatrix} 
        1 \\
        0
    \end{bmatrix}


The Levenberg–Marquardt Method
:::::::::::::::::::::::::::::::

Powell’s Dog Leg Method
:::::::::::::::::::::::::::::::






