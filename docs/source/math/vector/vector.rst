.. highlight:: c++

.. default-domain:: cpp

=======
Vector
=======

**definition**
-----------------

.. math:: \mathbf{a} = [a_1, a_2, a_3, \cdots , a_n]^T
 
 
**inner product**
------------------

.. math:: 

    \begin{align}
        \mathbf{a}^T \mathbf{b} &= \mathbf{b}^T \mathbf{a} \\
        &=
        \begin{bmatrix}
            a_1     \\
            a_2     \\
            \vdots  \\
            a_n
        \end{bmatrix}
        \begin{bmatrix}
            b_1, b_2, \cdots, b_n
        \end{bmatrix}  \\
        &=
        a_1 b_1 + a_2 b_2 + a_3 b_3 + \cdots + a_n b_n
    \end{align}

**out product**
----------------

.. math:: 

    \mathbf{a} 
    \times \mathbf{b} =  
    \left | 
        \begin{matrix}
            i   & j   &  k    \\
            a_1 & a_2 &  a_3  \\
            b_1 & b_2 &  b_3
        \end{matrix} 
    \right |


3维向量的叉乘写成矩阵方式：

.. math::

    \mathbf{a} \times \mathbf{b} =  
    \begin{bmatrix}
        0    & -a_3 & a_2 \\
        a_3  &  0   & -a_1 \\
        -a_2 &  a_1 & 0
    \end{bmatrix}
    \begin{bmatrix}
        b_1 \\
        b_2 \\
        b_3
    \end{bmatrix}
    

在机器人中更常见的写法是用^运算符号：

.. math::

    \mathbf{a}^{\wedge} \triangleq 
    \begin{bmatrix}
        0    & -a_3 & a_2 \\
        a_3  &  0   & -a_1 \\
        -a_2 &  a_1 & 0
    \end{bmatrix}

则容易证明有如下公式成立：

.. math::

    \mathbf{a} \times \mathbf{b} =  \mathbf{a}^{\wedge} \mathbf{b}

.. NOTE::

    * 性质1： :math:`\mathbf{a}^{\wedge} \mathbf{b} =  -\mathbf{b}^{\wedge} \mathbf{a}`
    * 性质2： :math:`\mathbf{a}^{\wedge} \mathbf{a} =  -\mathbf{I} + \mathbf{a}\mathbf{a}^T`



