## Math

## matrix definition

$m  \times n $ 维矩阵表示
$$
A_{m\times n} = 
    \begin{bmatrix}
        a_{11} & a_{12} & \cdots & a_{1m} \\
        a_{21} & a_{22} & \cdots & a_{2m} \\
        \vdots & \vdots & \cdots & \vdots \\
        a_{n1} & a_{n2} & \cdots & a_{mn}
    \end{bmatrix}_{m\times n}
$$

## matrix operation

$3  \times 3 $ dimension matrix ADD :
$$
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
$$
$3  \times 3 $ dimension matrix MINUS:
$$
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
$$
$3  \times 3 $ dimension matrix MULTIPLY:
$$
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
$$


## Schur Complements

设 $M$ 是一个 $n × n $矩阵，写成 $2 × 2 $块矩阵
$$
M =
\begin{bmatrix}
	A & B \\
	C & D
\end{bmatrix}
$$
其中 $A$ 是一个 $p × p$ 矩阵，$D$ 是一个$ q × q$ 矩阵，其中 $n = p + q$ （所以，$B $是一个$ p × q $矩阵$C$ 是一个 $q × p$ 矩阵）。 

我们可以尝试求解线性系统：
$$
\begin{bmatrix}
	A & B \\
	C & D
\end{bmatrix}
\begin{bmatrix}
	x \\
	y
\end{bmatrix}
=
\begin{bmatrix}
	c \\
	d
\end{bmatrix}
$$
那么
$$
Ax + By = c \\
Cx + Dy = d
$$
通过模仿高斯消元法，即假设 $D$ 是可逆的，我们首先求解 $y$ 得到：
$$
y = D^{-1}(d - Cx)
$$
在第一个方程中用这个表达式代替 y 后，我们得到：
$$
Ax + B(D^{-1}(d - Cx)) = c
$$
那么
$$
(A - BD^{-1}C)x = c - BD^{-1}d
$$
如果矩阵$ A - BD^{-1}C$是可逆的，那么我们得到系统的解
$$
\begin{aligned}
	x &=  (A - BD^{-1}C)^{-1}(c - BD^{-1}d) \\
	y &= D^{-1}(d - C(A-BD^{-1}C)^{-1}(c - BD^{-1}d))
\end{aligned}
$$
矩阵 $A − BD^{−1}C$ 称为 $D$ 在 $M$ 中的舒尔补。如果 $A$ 是可逆的，那么通过首先使用第一个方程消除 $x$ 我们发现 $A $在 $M$ 中的舒尔补是 $D − CA^{−1}B$ ( 这对应于当 $C = B^{T}$时 Boyd 和 Vandenberghe 中定义的 Schur 补

上述方程写为:
$$
\begin{aligned}
	x &=  (A - BD^{-1}C)^{-1}c - (A - BD^{-1}C)^{-1}BD^{-1}d \\
	y &= -D^{-1}C(A-BD^{-1}C)^{-1}c + (D^{-1} + D^{-1}C(A - BD^{-1}C)BD^{-1})d
\end{aligned}
$$
根据 $M$ 中 $D$ 的 Schur 补，产生 $M $的逆的公式，即
$$
\begin{bmatrix}
	A & B \\
	C & D
\end{bmatrix}^{-1}
=
\begin{bmatrix}
	(A - BD^{-1}C)^{-1} & -(A - BD^{-1}C)^{-1}BD^{-1} \\
	-D^{-1}C(A-BD^{-1}C)^{-1} & D^{-1} + D^{-1}C(A - BD^{-1}C)BD^{-1}
\end{bmatrix}
$$
片刻的反思
$$
\begin{bmatrix}
	A & B \\
	C & D
\end{bmatrix}^{-1}
=
\begin{bmatrix}
	(A - BD^{-1}C)^{-1} & 0 \\
	-D^{-1}C(A-BD^{-1}C)^{-1} & D^{-1}
\end{bmatrix}
\begin{bmatrix}
	I & -BD^{-1} \\
	0 & I
\end{bmatrix}
$$
然后
$$
\begin{bmatrix}
	A & B \\
	C & D
\end{bmatrix}^{-1}
=
\begin{bmatrix}
	I & 0 \\
	-D^{-1}C & I
\end{bmatrix}
\begin{bmatrix}
	(A - BD^{-1}C)^{-1} & 0 \\
	0 & D^{-1}
\end{bmatrix}
\begin{bmatrix}
	I & -BD^{-1} \\
	0 & I
\end{bmatrix}
$$
紧接着就是
$$
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
	A - BD^{-1}C & 0 \\
	0 & D
\end{bmatrix}
\begin{bmatrix}
	I & 0 \\
	D^{-1}C & I
\end{bmatrix}
$$
上式可以直接查，优点是只要求$D$的可逆性。

备注：如果 $A $是可逆的，那么我们可以使用 $A$ 的 Schur 补码 $D − CA^{−1}B$ 来获得 $M $的以下因式分解：
$$
\begin{bmatrix}
	A & B \\
	C & D
\end{bmatrix}
=
\begin{bmatrix}
	I & 0 \\
	CA^{-1} & I
\end{bmatrix}
\begin{bmatrix}
	A & 0 \\
	0 & D - CA^{-1}B
\end{bmatrix}
\begin{bmatrix}
	I & A^{-1}B \\
	0 & I
\end{bmatrix}
$$
如果 $D−CA^{−1}B$ 是可逆的，我们可以将上面的所有三个矩阵求逆，我们可以得到另一个公式，用 $(D−CA−1B) $表示 $M $的逆，即
$$
\begin{bmatrix}
	A & B \\
	C & D
\end{bmatrix}^{-1}
=
\begin{bmatrix}
	A^{-1} + A^{-1}B(D- CA^{-1}B)CA^{-1} & -A^{-1}B(D- CA^{-1}B)CA^{-1} \\
	-(D- CA^{-1}B)CA^{-1} & (D- CA^{-1}B)^{-1}
\end{bmatrix}
$$
如果 $A， D $和两个 Schur 补 $A − BD^{−1}C $和 $D − CA−1B $都是可逆的，则比较 $M^{−1}$ 的两个表达式，我们得到（非显而易见的）公式
$$
(A - BD^{-1}C)^{-1} = A^{-1} + A^{-1}B(D- CA^{-1}B)CA^{-1}
$$
使用这个公式，我们得到了另一个涉及$ A$  和 $ D$  的 Schur 补码的 $ M$  逆的表达式
$$
\begin{bmatrix}
	A & B \\
	C & D
\end{bmatrix}^{-1}
=
\begin{bmatrix}
	(A - BD^{-1}C)^{-1} & -A^{-1}B(D-CA^{-1}B)^{-1}) \\
	-(D- CA^{-1}B)CA^{-1} & (D- CA^{-1}B)^{-1}
\end{bmatrix}\
$$
如果我们设置 $D = I $并将$ B $更改为$ -B $我们得到
$$
(A + BC)^{-1} = A^{-1} - A^{-1}B(I - CA^{-1}B)^{-1}CA^{-1}
$$
称为矩阵求逆引理的公式

