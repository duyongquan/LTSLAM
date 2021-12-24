.. highlight:: c++

.. default-domain:: cpp

=======
Matrix
=======

**matrix definition**
----------------------

m :math:`\times` n维矩阵表示:

.. math:: 

    A_{m\times n} = 
    \begin{bmatrix}
        a_{11} & a_{12} & \cdots & a_{1m} \\
        a_{21} & a_{22} & \cdots & a_{2m} \\
        \vdots & \vdots & \cdots & \vdots \\
        a_{n1} & a_{n2} & \cdots & a_{mn}
    \end{bmatrix}_{m\times n}



**matrix operation**
----------------------

:math:`3\times 3` dimension matrix ADD：

.. math::

    A_{3 \times 3} + B_{3 \times 3} = 
    \begin{bmatrix}
        a_{11}    & a_{12} & a_{13} \\
        a_{21}    & a_{22} & a_{23} \\
        a_{31}    & a_{32} & a_{33} 
    \end{bmatrix}_{3 \times 3}
    + 
    \begin{bmatrix}
        b_{11}    & b_{12} & b_{13} \\
        b_{21}    & b_{22} & b_{23} \\
        b_{31}    & b_{32} & b_{33} 
    \end{bmatrix}_{3 \times 3}
    =
    \begin{bmatrix}
        a_{11}+b_{11}    & a_{12}+b_{12}  & a_{13}+b_{13}  \\
        a_{21}+b_{21}    & a_{22}+b_{22}  & a_{23}+b_{23}  \\
        a_{31}+b_{31}    & a_{32}+b_{32}  & a_{33}+b_{33}  
    \end{bmatrix}_{3 \times 3}

:math:`3\times 3` dimension matrix< MINUS：

.. math:: 

    A_{3 \times 3} - B_{3 \times 3} = 
    \begin{bmatrix}
    a_{11}    & a_{12} & a_{13} \\
    a_{21}    & a_{22} & a_{23} \\
    a_{31}    & a_{32} & a_{33} 
    \end{bmatrix}_{3 \times 3}
    - 
    \begin{bmatrix}
    b_{11}    & b_{12} & b_{13} \\
    b_{21}    & b_{22} & b_{23} \\
    b_{31}    & b_{32} & b_{33} 
    \end{bmatrix}_{3 \times 3}
    =
    \begin{bmatrix}
    a_{11}-b_{11}    & a_{12}-b_{12}  & a_{13}-b_{13}  \\
    a_{21}-b_{21}    & a_{22}-b_{22}  & a_{23}-b_{23}  \\
    a_{31}-b_{31}    & a_{32}-b_{32}  & a_{33}-b_{33}  
    \end{bmatrix}_{3 \times 3}




:math:`3\times 3` dimension matrix MULTIPLY：

.. math::

    \begin{align}
    A_{3 \times 3}  B_{3 \times 3} 
    &= 
    \begin{bmatrix}
    a_{11}    & a_{12} & a_{13} \\
    a_{21}    & a_{22} & a_{23} \\
    a_{31}    & a_{32} & a_{33} 
    \end{bmatrix}_{3 \times 3}
    \begin{bmatrix}
    b_{11}    & b_{12} & b_{13} \\
    b_{21}    & b_{22} & b_{23} \\
    b_{31}    & b_{32} & b_{33} 
    \end{bmatrix}_{3 \times 3}  \\
    &=
    \begin{bmatrix}
    a_{11}b_{11} + a_{12} b_{21}+ a_{13} b_{31}   & a_{11}b_{12} + a_{12} b_{22}+ a_{13} b_{32} & a_{11}b_{13} + a_{12} b_{23}+ a_{13} b_{33} \\
    a_{21}b_{11} + a_{22} b_{21}+ a_{23} b_{31}   & a_{21}b_{12} + a_{22} b_{22}+ a_{23} b_{32} & a_{21}b_{13} + a_{22} b_{23}+ a_{23} b_{33} \\
    a_{31}b_{11} + a_{32} b_{21}+ a_{33} b_{31}   & a_{31}b_{12} + a_{32} b_{22}+ a_{33} b_{32} & a_{31}b_{13} + a_{32} b_{23}+ a_{33} b_{33} \\
    \end{bmatrix}_{3 \times 3}
    \end{align}




**schur compliment**
----------------------

Let :math:`M` be an :math:`n × n` matrix written a as :math:`2 × 2` block matrix

.. math:: 

    M = 
    \begin{bmatrix}
        A & B \\
        C & D
    \end{bmatrix}



matrix decomposition：

.. math::

    \begin{bmatrix}
        A & B \\
        C & D
    \end{bmatrix}
    =
    \begin{bmatrix}
        I & BD^{-1} \\
        0 & I
    \end{bmatrix}
    \begin{bmatrix}
        (A − BD^{−1}C)   & 0 \\
        0                & D
    \end{bmatrix}
    \begin{bmatrix}
        I       & 0 \\
        D^{-1}C & I
    \end{bmatrix}