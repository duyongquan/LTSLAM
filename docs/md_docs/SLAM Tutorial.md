 

<center> <font color='green' size='15'> SLAM Tutorial</font> </center>

 

# Part I Math

## 1 Vector

### 1.1 vector definition


$$
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
$$


n维向量表示：
$$
\mathbf{a} = [a_1, a_2, a_3, \cdots , a_n]^T
$$

### 1.2 vector inner product

$$
\begin{align}
\mathbf{a}^T\mathbf{b} &= \mathbf{b}^T\mathbf{a} \\
 &= 
 
 \begin{bmatrix}
 a_1 \\
 a_2 \\
 \vdots \\
 a_n
 \end{bmatrix}

 \begin{bmatrix}
 b_1, b_2, \cdots, b_n
 \end{bmatrix}  \\
 &=
 a_1b_1 + a_2b_2 + a_3b_3 + \cdots + a_nb_n
\end{align}
$$

### 1.3 vector  out product

3维向量的叉乘形式：
$$
\mathbf{a} \times \mathbf{b} =  

\left | \begin{matrix}
 i   & j   & k \\
 a_1 & a_2 & a_3  \\
 b_1 & b_2 &  b_3\
\end{matrix} \right |
$$
3维向量的叉乘写成矩阵方式：
$$
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
$$
在机器人中更常见的写法是用${\wedge}$运算符号：
$$
\mathbf{a}^{\wedge} \triangleq 
\begin{bmatrix}
0    & -a_3 & a_2 \\
a_3  &  0   & -a_1 \\
-a_2 &  a_1 & 0
\end{bmatrix}
$$
则容易证明有如下公式成立：
$$
\mathbf{a} \times \mathbf{b} =  \mathbf{a}^{\wedge} \mathbf{b}
$$

* 性质1： $\mathbf{a}^{\wedge} \mathbf{b} =  -\mathbf{b}^{\wedge} \mathbf{a} $
* 性质2：$\mathbf{a}^{\wedge} \mathbf{a} =  -\mathbf{I} + \mathbf{a}\mathbf{a}^T $

## 2 quaternion

### definition

$$
\mathbf{q} = q_w + q_x + q_y + q_z
$$



### 2.1 四元数的和

$$
\mathbf{p} + \mathbf{q} = \begin{bmatrix} p_w \ \mathbf{p}_v \end{bmatrix} +

\begin{bmatrix} q_w \ \mathbf{q}_v \end{bmatrix} =

\begin{bmatrix} p_w + q_w \ \mathbf{p}_v + \mathbf{q}_v \end{bmatrix} =

\begin{bmatrix} p_w + q_w \ \mathbf{p}_x + \mathbf{q}_x \ \mathbf{p}_y + \mathbf{q}_y \ \mathbf{p}_z + \mathbf{q}_z \end{bmatrix}
$$

对应的元素直接相加即可。



### 2.2 四元数的乘积

$$
\begin{align}
\mathbf{p} \otimes \mathbf{q} &=

\begin{bmatrix} p_w & \mathbf{p}_x & \mathbf{p}_y & \mathbf{p}_z \end{bmatrix} 

\begin{bmatrix} q_w & \mathbf{q}_x & \mathbf{q}_y & \mathbf{q}_z \end{bmatrix} = 

\begin{bmatrix} 
p_w q_w -p_x q_x - p_y q_y -p_z q_z \\ 
p_w q_x +p_x q_w + p_y q_z -p_z q_y \\ 
p_w q_y -p_x q_y + p_y q_w +p_z q_x \\
p_w q_z +p_x q_z - p_y q_x +p_z q_w 
\end{bmatrix}
\end{align}
$$

写成标量和向量形式： 
$$
\mathbf{p} \otimes \mathbf{q} =

\begin{bmatrix} q_w & \mathbf{q}_x & \mathbf{q}_y & \mathbf{q}_z \end{bmatrix} =

\begin{bmatrix} p_w q_w - \mathbf{p}^T_v\mathbf{q}_v \ p_w \mathbf{q}_v + q_w \mathbf{p}_v +\mathbf{p}_v \times \mathbf{q}_v \end{bmatrix}
$$
$$  四元数不满足交换律 : $$
$$
\mathbf{p} \otimes \mathbf{q} \ne \mathbf{q} \otimes \mathbf{p}
$$
 $$ 结合律： $$ 
$$
(\mathbf{p} \otimes \mathbf{q}) \otimes \mathbf{r}= \mathbf{q} \otimes (\mathbf{p} \otimes \mathbf{r})
$$

### 2.3 矩阵形式

$$
\mathbf{q}_1 \otimes \mathbf{q}_2 = \mathbf{Q}^{+}\mathbf{q}_2 \ \mathbf{q}_1 \otimes \mathbf{q}_2 = \mathbf{Q}^{-}\mathbf{q}_1
$$

其中： 
$$
\mathbf{Q}^{+} = q_w\mathbf{I} +

\begin{bmatrix} 0 & -\mathbf{q}_v^{T} \mathbf{q}_v & [\mathbf{q}*v]_{\times}

\end{bmatrix} 

\quad\quad\quad\quad

\mathbf{Q}^{-} =

q_w\mathbf{I} + \begin{bmatrix}
0 & -\mathbf{q}_v^{T} \ \mathbf{q}_v & -[\mathbf{q}*v]*{\times}

\end{bmatrix}
$$

### 2.4 单位四元数

$$
\mathbf{q}_{1} = 1 =

\begin{bmatrix} 1 \ \mathbf{0}_x \end{bmatrix}
$$

### 2.5 四元数共轭

共轭的定义： 
$$
\mathbf{q}^{\star} \triangleq q_w - \mathbf{q}_v =

\begin{bmatrix} q_w \ - \mathbf{q}_v \end{bmatrix}
$$

### 2.6 四元数的模

$$
\mathbf{q}| = \sqrt{q_w^2 + q_x^2 + q_y^2 + q_z^2}
$$

### 2.7 四元数的逆

$$
\mathbf{q} \otimes \mathbf{q}^{-1} = \mathbf{q}^{-1} \otimes \mathbf{q} = \mathbf{q}_{1}
$$

### 2.8 旋转表示

$$
\mathbf{q}({\theta}) =

\cos{\frac{\theta}{2}} + \mathbf{u}\sin{\frac{\theta}{2}} =

\begin{bmatrix} cos{\frac{\theta}{2}}\ \mathbf{u}\sin{\frac{\theta}{2}} \end{bmatrix}
$$

### 2.9 向量旋转

$$
\begin{bmatrix} 0 \ \mathbf{v}^{\prime} \end{bmatrix} =

\mathbf{q} \otimes

\begin{bmatrix} 0 \ \mathbf{v} \end{bmatrix} \otimes

\mathbf{q}^{\star}
$$



### 3.0 旋转矩阵

$$
\mathbf{v}^{\prime} = \mathbf{R}\mathbf{v}
$$

其中： 
$$
\mathbf{R}{q} = 
\begin{bmatrix} 
q_w^2 + q_x^2 - q_y^2 - q_z^2 & 2(q_xq_y - q_wq_z) & 2(q_xq_z + q_wq_y) \\
2(q_xq_y + q_wq_z) & q_w^2 - q_x^2 + q_y^2 - q_z^2 & 2(q_yq_z - q_wq_x) \\
2(q_xq_z - q_wq_y) & 2(q_yq_z + q_wq_x) & q_w^2 - q_x^2 - q_y^2 + q_z^2 
\end{bmatrix}
$$


### 3.1 导数

$$
\mathbf{\dot{q}} = \frac{1}{2} \mathbf{q} \otimes \mathbf{w}
$$

## 2 Matrix

## 2.1 matrix definition

m$\times$n维矩阵表示:
$$
A_{m\times n} = 
\begin{bmatrix}
a_{11} & a_{12} & \cdots & a_{1m} \\
a_{21} & a_{22} & \cdots & a_{2m} \\
\vdots & \vdots & \cdots & \vdots \\
a_{n1} & a_{n2} & \cdots & a_{mn}

\end{bmatrix}_{m\times n}
$$

### 2.2 matrix operation

* $3\times 3$ dimension matrix <font color='red'>add</font>
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

* $3\times 3$ dimension matrix<font color='red'> minus</font>
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

* $3\times 3$ dimension matrix <font color='red'>multiply</font>
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

### 2.3 schur compliment

Let $M$ be an $ n × n$ matrix written a as $2 × 2$ block matrix
$$
M = 
\begin{bmatrix}
A & B \\
C & D
\end{bmatrix}
$$
where $A$ is a $p × p$  matrix and $D$ is a $q × q$ matrix, with $n = p + q$ (so, $B$ is a $p × q$ matrix and $C$ is a $q × p$ matrix). We can try to solve the linear system
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
that is
$$
\begin{align}
Ax + By = c\\
Cx + Dy = d
\end{align}
$$
by mimicking Gaussian elimination, that is, assuming that $D$ is invertible, we first solve for y getting
$$
y = D^{-1}(d − Cx)
$$
and after substituting this expression for y in the first equation, we get
$$
Ax + B(D^{−1}(d − Cx)) = c
$$
that is
$$
 (A − BD^{−1}C)x = c − BD^{−1}d
$$
If the matrix $A − BD^{−1}C$ is invertible, then we obtain the solution to our system
$$
\begin{align}
x &= (A − BD^{−1}C)^{−1}(c − BD^{−1}d) \\
y &= D^{−1}(d − C(A − BD^{−1}C)^{−1}(c − BD^{−1}d))
\end{align}
$$
The matrix, $A − BD^{−1}C$, is called the <font color='green'>Schur Complement</font> of $D$ in $M$. If $A$ is invertible, then by eliminating $x$ first using the first equation we find that the Schur complement of $A$ in $M$ is $D − CA^{−1}B$ . The above equations written as

The above equations written as
$$
\begin{align}
x &= (A − BD^{−1}C)^{−1}c − (A − BD^{−1}C)^{−1}BD^{−1}d \\
y &= −D^{−1}C(A − BD^{−1}C)^{−1}c + (D^{−1} + D^{−1}C(A − BD^{−1}C)^{−1}BD^{−1})d
\end{align}
$$
yield a formula for the inverse of $M$ in terms of the Schur complement of $D$ in $M$, namely
$$
\begin{bmatrix}
A & B \\
C & D
\end{bmatrix}^{-1}
=
\begin{bmatrix}
(A − BD^{−1}C)^{-1}   & -(A − BD^{−1}C)^{-1}BD^{-1} \\
-DC(A − BD^{−1}C)^{-1} & D^{-1} + D^{-1}C(A − BD^{−1}C)^{-1}CD^{-1}
\end{bmatrix}
$$
A moment of reflexion reveals that
$$
\begin{bmatrix}
A & B \\
C & D
\end{bmatrix}^{-1}
=
\begin{bmatrix}
(A − BD^{−1}C)^{-1}   & 0 \\
-DC(A − BD^{−1}C)^{-1} & D^{-1}
\end{bmatrix}
\begin{bmatrix}
I & − BD^{−1} \\
0 & I
\end{bmatrix}
$$
and then
$$
\begin{bmatrix}
A & B \\
C & D
\end{bmatrix}^{-1}
=
\begin{bmatrix}
I & 0 \\
− D^{−1}C & I
\end{bmatrix}

\begin{bmatrix}
(A − BD^{−1}C)^{-1}   & 0 \\
0                     & D^{-1}
\end{bmatrix}
\begin{bmatrix}
I & − BD^{−1} \\
0 & I
\end{bmatrix}
$$
It follows immediately that
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
(A − BD^{−1}C)   & 0 \\
0                     & D
\end{bmatrix}
\begin{bmatrix}
I & 0 \\
D^{-1}C & I
\end{bmatrix}
$$
The above expression can be checked directly and has the advantage of only requiring the invertibility of $D$

### 2.4 matrix derivatives



### 2.5 Function

$$
\begin{align}
e^{x}   &= 1 + x + \frac{1}{2!}x^2 + \frac{1}{3!}x^3 + \cdots \\
\sin{x} &= x - \frac{1}{3!}x^3 + \frac{1}{5!}x^5 + \cdots \\
\cos{x} & = 1 -  \frac{1}{2!}x^2 + \frac{1}{4!}x^4+ \cdots \\

\end{align}
$$



## 3 Probability

### 3.1 one dimension gaussian

<img src='./images/probility/normal-distribution-curve-1.png'>

一维正态分布 :
$$
p(x) = 
\frac{1}{\sqrt{2\pi \sigma^2}}
\exp
\left \{
	-\frac{1}{2}\ 
	\frac{(x - \mu)^2}{\sigma^2}
\right \}
$$



### 3.2 two dimension gaussian

<img src='./images/probility/MultivariateNormal.png'>

二维正态分布 :
$$
p(x,y) = 
\frac{1}{2\pi \sigma_{1} \sigma_{2}} 
\exp
\left \{
	-\frac{1}{2}\ 
	\frac{(x - \mu_{1})^2}{\sigma_{1}^2} +
	\frac{(y - \mu_{1})^2}{\sigma_{2}^2} 
\right \}
$$

### 3.3 mean and variance





### 3.4  bayes network



### 3.5 markov network



### 3.6 marginalization



## 4 least sqaure method

###  4.1 definition

$$
\min_{x}\,\, Ax = b \\ 
$$

### 4.2  SVD



![](/home/quan/workspace/slam/tutorial/SLAMLearn/images/svd.png)

任何一个矩阵$A$都可以被分解成 $A = U\Sigma V^T$的形式，其中$U$和$V$都是正交矩阵，且满足以下性质：
$$
\begin{align}
U^TU = I \\
V^TV = I
\end{align}
$$
SVD求解$Ax = b$ ：
$$
\begin{align}


\end{align}
$$


### 4.3 PCG

 



### 4.4 QR



## 5 ODE



## 6 Taylor's Formula

### 6.1 unary Taylor's formula

$$
f(x) = \frac{f(x_0)}{0!} + \frac{f^{\prime}(x_0)}{1!}(x- x_0) + \frac{f^{\prime{\prime}}(x_0)}{2!}(x- x_0)^2 + \cdots + \frac{f^{(n)}(x_0)}{n!}(x- x_0)^n + R_n(x)
$$

其中： $R_n(x) = \frac{f^{(n+1)}(\epsilon)}{(n+1)!}(x- x_0)^{n+1} $为$n$阶泰勒余项。

### 6.2 multivariate Taylor's formula

$$
f(x, y) = \frac{f(x_0, y_0)}{0!} + \frac{\partial f_x(x_0, y_0)}{1!}(x- x_0) + \frac{f_y(x_0, y_0)}{1!}(y- y_0) + 
\frac{\partial f_{xx}{(x_0, y_0)}}{2!}(x- x_0)^2 +  \\
\frac{\partial f_{yy}{(x_0, y_0)}}{2!}(y- y_0)^2 +
\partial f_{xy}{(x_0, y_0)}(x- x_0)(y- y_0) 
\cdots
$$

# Part II SLAM base on Filter

### KF

### EKF

### UKF

### OpenVins



# Part III SLAM base on Graph

### IMU Sensor

<img src='./images/vins-mono/imu.png'>

$IMU$传感器输出的测量数据：

* 加速度$a_m$
* 角速度$w_m$

#### IMU Model

$$
\begin{align}
	\tilde{\mathbf{\omega}}^b_m &= \mathbf{\omega}^b + \mathbf{b}^g + \mathbf{n}^g\\
	\tilde{a}^b_m &= \mathbf{q_{b\omega}}(\mathbf{a}^{\omega} +\mathbf{g}^{\omega}) + \mathbf{b}^a + \mathbf{n}^a

\end{align}
$$

where : 

* $b$表示imu机体坐标系body
* $w$表示世界坐标系world
* 上标$g$表示gyro
* $a$表示acc



### Camera Sensor

#### Camera model

### VINS-Mono

#### 误差状态(error-state)的状态方程



误差状态方程:
$$
\delta{X}_{k+1} = (\mathbf{I} + \mathbf{F}\delta{t})\delta{X}_{k} + (\mathbf{G}\delta{t})\mathbf{n}
$$

$$
\underbrace
{
    \begin{bmatrix}
        \delta\alpha_{k+1} \\
        \delta\theta_{k+1} \\
        \delta\beta_{k+1}  \\
        \delta{b_a}_{k+1}  \\
        \delta{b_g}_{k+1} 
    \end{bmatrix}
}_{\delta{X}_{k+1}} 
=
\underbrace
{
    \begin{bmatrix}
        I & \mathbf{f}_{01} &  \delta{\mathbf{t}} & -\frac{1}{4}(q_k + q_{k+1}\delta{t^2}) &  \mathbf{f}_{04} \\
        
       0  & I - [\frac{w_{k} + w_{k+1}}{2} - b_{wk}]_{\times}\delta{t} & 0 & 0 & -\delta{t} \\
       
        0 &  \mathbf{f}_{21}  & I & -\frac{1}{2}(q_k + q_{k+1}\delta{t}) & \mathbf{f}_{24}\\
        
        0 & 0 & 0 & I & 0  \\
        0 & 0 & 0 & 0 & I   
    \end{bmatrix}
}_{\mathbf{I} + \mathbf{F}\delta{t}} 
\underbrace
{
    \begin{bmatrix}
        \delta\alpha_{k} \\
        \delta\theta_{k} \\
        \delta\beta_{k}  \\
        \delta{b_a}_{k}  \\
        \delta{b_g}_{k} 
    \end{bmatrix}
}_{\delta{X}_{k}} 
+ \\
\underbrace
{
    \begin{bmatrix}
        \frac{1}{4}q_k\delta{t}^2 & v_{01} & \frac{1}{4}q_{k+1}\delta{t}^2 & v_{03} & 0 & 0 \\
        
        0 & \frac{1}{2}\delta{t} & 0 & \frac{1}{2}\delta{t} & 0 & 0\\
        \frac{1}{2}q_k \delta{t} & v_{21} & \frac{1}{2}{q_k+1}\delta{t} &  v_{23} & 0 & 0 \\
        0 & 0 & 0 & 0 & \delta{t} & 0 \\
        
        0 & 0 & 0 & 0 & 0 & \delta{t}
    \end{bmatrix}
}_{\mathbf{G}\delta{t}}
\underbrace
{
    \begin{bmatrix}
        n_{a0} \\
        n_{w0} \\
        n_{a1} \\
        n_{w1} \\
        n_{ba} \\
        n_{bg} 
    \end{bmatrix}
}_{\mathbf{n}}
$$
其中：
$$
\begin{align}
\mathbf{f}_{01} &=  -\frac{1}{4}(-q_{k}[a_{k} - b_{ak}]_{\times}\delta{t}^2)\delta{t} - \frac{1}{4}(q_{k+1}[a_{k+1} - b_{ak}]_{\times}(\mathbf{I} - [\frac{w_{k+1} + w_{k}}{2} - b_{gk}]_{\times}\delta{t})\delta{t}^2 \\

\mathbf{f}_{21} &=  -\frac{1}{2}(-q_{k}[a_{k} - b_{ak}]_{\times}\delta{t})\delta{t} - \frac{1}{2}(q_{k+1}[a_{k+1} - b_{ak}]_{\times}(\mathbf{I} - [\frac{w_{k+1} + w_{k}}{2} - b_{gk}]_{\times}\delta{t})\delta{t}  \\

\mathbf{f}_{04} &=  \frac{1}{4}(-q_{k+1}[a_{k+1} - b_{ak}]_{\times}\delta{t}^2)(-\delta{t}) \\

\mathbf{f}_{24} &= \frac{1}{2}(-q_{k+1}[a_{k+1} - b_{ak}]_{\times}\delta{t})(-\delta{t})  \\

v_{01} &=  \frac{1}{4}(-q_{k+1}[a_{k+1} - b_{ak}]_{\times}\delta{t}^2)\delta{t}\\

v_{03} &=  \frac{1}{4}(-q_{k+1}[a_{k+1} - b_{ak}]_{\times}\delta{t}^2)\frac{1}{2}\delta{t}\\

v_{21} &=  \frac{1}{2}(-q_{k+1}[a_{k+1} - b_{ak}]_{\times}\delta{t}^2)\frac{1}{2}\delta{t}\\

v_{23} &=  \frac{1}{2}(-q_{k+1}[a_{k+1} - b_{ak}]_{\times}\delta{t}^2)\frac{1}{2}\delta{t}

\end{align}
$$

**<font color='red'> integration_base.h </font>**代码如下:

```c++
MatrixXd F = MatrixXd::Zero(15, 15);
F.block<3, 3>(0, 0) = Matrix3d::Identity();
F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt + 
    -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * 
    (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
F.block<3,3>(0,9) =-0.25(delta_q.toRotationMatrix()+result_delta_q.toRotationMatrix())*_dt*_dt;
F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt + 
    -0.5 *result_delta_q.toRotationMatrix() * R_a_1_x *(Matrix3d::Identity()-R_w_x * _dt)* _dt;
F.block<3, 3>(6, 6) = Matrix3d::Identity();
F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix()+result_delta_q.toRotationMatrix())*_dt;
F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
F.block<3, 3>(9, 9) = Matrix3d::Identity();
F.block<3, 3>(12, 12) = Matrix3d::Identity();

MatrixXd V = MatrixXd::Zero(15,18);
V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() *R_a_1_x * _dt*_dt* 0.5 * _dt;
V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;
```



####  单目相机初始化

<img src='./images/vins-mono/system_initialize.png'>





相机坐标系 $c_0$ 为世界坐标系:

本体坐标系转换到 $c_0$坐标系:
$$
\begin{align}
	q_{b_k}^{c_0} &= q_{c_k}^{c_0}\otimes (q_{c}^{b})^{-1} \\
	sp_{b_k}^{c_0} &= s\bar{p}_{c_k}^{c_0} - R_{b_k}^{c_0}p_{c}^{b} \\
\end{align}
$$
其中参数$s$ 给视觉测量的位移赋予尺度信息。



#### SFM与 IMU预积分结果对齐 

##### 陀螺仪偏置的校正 

校正陀螺仪偏置的目标函数为:
$$
\min_{\small_{\delta{b_w}}} \sum_{k\in B} ||(q_{b_{k+1}}^{c_0})^{-1} \otimes q_{b_{k}}^{c_0} \otimes {\gamma}_{b_{k+1}}^{b_k}||^{2}
$$
其中：

* $B$表示滑动窗口中的所有关键帧

* $\gamma_{b_{k+1}}^{b_k} = \hat{\gamma}_{b_{k+1}}^{b_k} \otimes 
  \begin{bmatrix}
  	1 \\
  	\frac{1}{2}J_{\delta{b_{w_k}}}^{\delta{\theta_{b_{k+1}}^{b_k}}} \delta{b}_{w_k}
  \end{bmatrix}$

* ${\gamma}_{b_{k+1}}^{b_k}$关于陀 螺仪偏置误差$\delta{b}_w$的一阶近似。

  

推导：
$$
\begin{align}
	(q_{b_{k+1}}^{c_0})^{-1} \otimes q_{b_{k}}^{c_0} \otimes {\gamma}_{b_{k+1}}^{b_k} 
	&= 
	\begin{bmatrix}
		1 \\
		0 \\
		0 \\
		0
	\end{bmatrix}
	\\
	
	 \hat{\gamma}_{b_{k+1}}^{b_k} \otimes 
        \begin{bmatrix}
            1 \\
            \frac{1}{2}J_{\delta{b_{w_k}}}^{\delta{\theta_{b_{k+1}}^{b_k}}} \delta{b}_{w_k}
        \end{bmatrix}
        
        &= (q_{b_{k+1}}^{c_0})^{-1} \otimes q_{b_{k}}^{c_0} \otimes 
        \begin{bmatrix}
        	1 \\
        	0 \\
        	0 \\
        	0
        \end{bmatrix}
        
        \\
        
        \begin{bmatrix}
            1 \\
            \frac{1}{2}J_{\delta{b_{w_k}}}^{\delta{\theta_{b_{k+1}}^{b_k}}} \delta{b}_{w_k}
        \end{bmatrix}
        
        &= (\hat{\gamma}_{b_{k+1}}^{b_k})^{-1} \otimes (q_{b_{k+1}}^{c_0})^{-1} \otimes q_{b_{k}}^{c_0} \otimes 
        \begin{bmatrix}
        	1 \\
        	0 \\
        	0 \\
        	0
        \end{bmatrix}
\end{align}
$$
只考虑虚部，则有:
$$
\begin{align}
    \frac{1}{2}J_{\delta{b_{w_k}}}^{\delta{\theta_{b_{k+1}}^{b_k}}} 
    \delta{b}_{w_k}
    &= 
    (\hat{\gamma}_{b_{k+1}}^{b_k})^{-1} \otimes 
    (q_{b_{k+1}}^{c_0})^{-1} 		    \otimes 
    q_{b_{k}}^{c_0} 
    \\      
    
    \Rightarrow
    \quad 
    
    \underbrace{
    	J_{\delta{b_{w_k}}}^{\delta{\theta_{b_{k+1}}^{b_k}}} 		
    	\delta{b}_{w_k}}_{Ax} 
    &=
    2
    \underbrace{
    	(\hat{\gamma}_{b_{k+1}}^{b_k})^{-1} \otimes 
    	(q_{b_{k+1}}^{c_0})^{-1} 			\otimes 
    	q_{b_{k}}^{c_0} }_{b}  
    \\
    \Rightarrow
    \quad 
    
    \underbrace{
    	(J_{\delta{b_{w_k}}}^{\delta{\theta_{b_{k+1}}^{b_k}}})^{T}
    	J_{\delta{b_{w_k}}}^{\delta{\theta_{b_{k+1}}^{b_k}}} 		
    	\delta{b}_{w_k}}_{A^{T}Ax} 
    &=
    2\underbrace{
    	(\hat{\gamma}_{b_{k+1}}^{b_k})^{-1} \otimes 
    	(q_{b_{k+1}}^{c_0})^{-1} 			\otimes 
    	q_{b_{k}}^{c_0} }_{A^{T}b}
    
\end{align}
$$
这样就可以使用 $cholesky$ 分解求解矩阵，获取目标函数达到最小的解$\delta{b_w}$ ，从而完成对陀螺仪偏置的校正。

**<font color='red'> initial_aligment.cpp </font>**代码如下:

```c++
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs)
{
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++){
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    delta_bg = A.ldlt().solve(b);
    ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose());

    // 因为求解出的Bias是变化量，所以要累加
    for (int i = 0; i <= WINDOW_SIZE; i++){
        Bgs[i] += delta_bg;
    }
    // 利用新的Bias重新repropagate
    for (frame_i = all_image_frame.begin(); next(frame_i)!= all_image_frame.end( ); frame_i++){
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
    }
}
```



##### 初始化速度、重力向量和尺度因子

待优化的变量为：
$$
\chi_{I} = 
\begin{bmatrix}
	\mathbf{v}_{b_0}^{b_0} \\
	\mathbf{v}_{b_0}^{b_0} \\
	\mathbf{v}_{b_0}^{b_0} \\
	\vdots 				   \\
	\mathbf{v}_{b_0}^{b_0} \\
	\mathbf{g}^{c_0} 	   \\
	s
\end{bmatrix}
$$
将预积分项广义的位置和速度中的世界坐标系$w$换为 $c_0$坐标系：
$$
\begin{align}

	\alpha_{b_{k+1}}^{b_{k}} &= 
		R_{c_{0}}^{b_{k}}(\mathbf{p}_{b_{k+1}}^{c_0} - 
		\mathbf{p}_{b_{k}}^{c_0} - 
		\mathbf{\upsilon}_{b_{k}}^{c_0} \Delta{t_k} + 
		\frac{1}{2}\mathbf{g}^{c_0} \Delta{t_k}^2)
	\\
	
	\beta_{b_{k+1}}^{b_{k}} &= 
		R_{c_{0}}^{b_{k}}(\mathbf{v}_{b_{k+1}}^{c_0} - 
		\mathbf{\upsilon}_{b_{k}}^{c_0} +
		\mathbf{g}^{c_0} \Delta{t_k})
\end{align}
$$
$\mathbf{p}_{b_{k+1}}^{c_0}$和$\mathbf{p}_{b_{k}}^{c_0}$可由视觉 $SFM$获得: 
$$
\begin{align}
	\mathbf{p}_{b_{k}}^{c_0} &= s\bar{\mathbf{p}}_{b_{k}}^{c_0} \\
	\mathbf{p}_{b_{k+1}}^{c_0} &=s\bar{\mathbf{p}}_{b_{k+1}}^{c_0}
\end{align}
$$
代入等式：
$$

$$




重力矢量的模长固定（$g = 9.8 m/s^2$），其为2个自由度，在切空间上对其参数化
$$
\begin{align}
	\hat{g} &= ||g|| \cdot \bar{\hat{g}} + w_1 \vec{b_1} + w_2 \vec{b_2} \\
			&= ||g|| \cdot \bar{\hat{g}} + B\vec{w}
\end{align} , \quad \quad B\in \mathbb{R}^{3\times2}, \vec{w} \in \mathbb{R}^{2\times1}
$$
令$\hat{g} = g^{c_0}$, 将其代入上一小节公式得 :
$$
\begin{align}
\begin{bmatrix}
	-I\Delta{t}_k & 0 & \frac{1}{2}R_{c_0}^{b_k}\Delta{t}_{k}^2B & R_{c_0}^{b_k}(\bar{p}_{c_{k+1}^{c_0}} - \bar{p}_{c_{k}^{c_0}} ) \\
	 
	-I & R_{c_0}^{b_k}R_{b_{k+1}}^{c_0} & R_{c_0}^{b_k}\Delta{t}_{k}B & 0
\end{bmatrix} 
\begin{bmatrix}
	v_{b_k}^{b_k} \\
	v_{b_{k+1}}^{b_{k+1}} \\
	\vec{w} \\
	s
\end{bmatrix} 
\\
=
\begin{bmatrix}
	\alpha_{b_{k+1}}^{b_k} - p_{c}^{b} + R_{c_0}^{b_k}R_{b_{k+1}}^{c_0} p_{c}^{b} - \frac{1}{2}R_{c_0}^{b_k} \Delta{t}_{k}^2 ||g||\cdot \bar{\hat{g}} \\
	
	
	\beta_{b_{k+1}}^{b_k} - R_{c_0}^{b_k} \Delta{t}_{k} ||g||\cdot \bar{\hat{g}}
\end{bmatrix}
\end{align}
$$

#### $IMU$测量残差


$IMU$测量残差$r_B(\hat{\mathbf{Z}}^{b_{k}}_{b_{k+1}}, \mathbf{\chi})$ ：
$$
r_B(\hat{\mathbf{Z}}^{b_{k}}_{b_{k+1}}, \mathbf{\chi})
=
\begin{bmatrix}
    \delta\alpha_{b_{k+1}}^{b_{k}} \\
    \delta\theta_{b_{k+1}}^{b_{k}} \\
    \delta\beta_{b_{k+1}}^{b_{k}}  \\
    \delta {\mathbf{b}}_{a}  \\
    \delta {\mathbf{b}}_{w}  \\
\end{bmatrix}
=
\begin{bmatrix}
	\mathbf{R}_{w}^{b_k}(\mathbf{p}_{b_{k+1}^{w}} - \mathbf{p}_{b_{k}^{w}} - 						\mathbf{\upsilon}_{b_{k}^{w}} \Delta{t_k} + \frac{1}{2}\mathbf{g}^w 					\Delta{t_k}^2) -  \hat{\alpha}_{b_{k+1}}^{b_{k}} \\
	
	2\begin{bmatrix}
		(\hat{\mathbf{\gamma}}_{b_{k+1}}^{b_{k}})^{-1} \otimes \mathbf{q}_{b_{k}^{w}} 				\otimes \mathbf{q}_{b_{k+1}^{w}}
	\end{bmatrix} \\
	
    \mathbf{R}_{w}^{b_k}(\mathbf{\upsilon}_{b_{k+1}^{w}} - 											\mathbf{\upsilon}_{b_{k}^{w}} + \mathbf{g}^w \Delta{t_k}) - 							\hat{\beta}_{b_{k+1}}^{b_{k}} \\
	
	\mathbf{b}_{ab_{k+1}} - \mathbf{b}_{wb_{k}} \\
	
	\mathbf{b}_{wb_{k+1}} - \mathbf{b}_{wb_{k}} \\
	
\end{bmatrix}
$$
对于两帧之间的$IMU$测量残差，优化变量为：
$$
\underbrace{
    \begin{bmatrix}
        \mathbf{p}_{b_{k}}^{w}, \mathbf{\theta}_{b_{k}}^{w} 
    \end{bmatrix}
    \quad 
    \begin{bmatrix}
        \mathbf{v}_{b_{k}}^{w}, \mathbf{b}_{a_{k}}, \mathbf{b}_{w_{k}} 
    \end{bmatrix}
}_{k}

\quad 
\underbrace{
    \begin{bmatrix}
        \mathbf{p}_{b_{k+1}}^{w}, \mathbf{\theta}_{b_{k+1}}^{w} 
    \end{bmatrix}
    \quad 
    \begin{bmatrix}
        \mathbf{v}_{b_{k+1}}^{w}, \mathbf{b}_{a_{k+1}}, \mathbf{b}_{w_{k+1}} 
    \end{bmatrix}
}_{k+1}
$$
雅克比矩阵$jacobians$ :
$$
J =
\begin{bmatrix}
	\frac{\partial{\delta\alpha_{b_{k+1}}^{b_{k}}}}{\mathbf{p}_{b_{k}}^{w}} & 				\frac{\partial{\delta\alpha_{b_{k+1}}^{b_{k}}}}{\mathbf{\theta}_{b_{k}}^{w}} & 
	\frac{\partial{\delta\alpha_{b_{k+1}}^{b_{k}}}}{\mathbf{v}_{b_{k}}^{w}} & 				\frac{\partial{\delta\alpha_{b_{k+1}}^{b_{k}}}}{\mathbf{b}_{a_{k}}}  &
	\frac{\partial{\delta\alpha_{b_{k+1}}^{b_{k}}}}{\mathbf{b}_{w_{k}}}  &
	\frac{\partial{\delta\alpha_{b_{k+1}}^{b_{k}}}}{\mathbf{p}_{b_{k+1}}^{w}} & 			\frac{\partial{\delta\alpha_{b_{k+1}}^{b_{k}}}}{\mathbf{\theta}_{b_{k+1}}^{w}} & 
	\frac{\partial{\delta\alpha_{b_{k+1}}^{b_{k}}}}{\mathbf{v}_{b_{k+1}}^{w}} & 			\frac{\partial{\delta\alpha_{b_{k+1}}^{b_{k}}}}{\mathbf{b}_{a_{k+1}}}  &
	\frac{\partial{\delta\alpha_{b_{k+1}}^{b_{k}}}}{\mathbf{b}_{w_{k+1}}} \\
	
	\frac{\partial{\delta\theta_{b_{k+1}}^{b_{k}}}}{\mathbf{p}_{b_{k}}^{w}} & 				\frac{\partial{\delta\theta_{b_{k+1}}^{b_{k}}}}{\mathbf{\theta}_{b_{k}}^{w}} & 
	\frac{\partial{\delta\theta_{b_{k+1}}^{b_{k}}}}{\mathbf{v}_{b_{k}}^{w}} & 				\frac{\partial{\delta\theta_{b_{k+1}}^{b_{k}}}}{\mathbf{b}_{a_{k}}}  &
	\frac{\partial{\delta\theta_{b_{k+1}}^{b_{k}}}}{\mathbf{b}_{w_{k}}}  &
	\frac{\partial{\delta\theta_{b_{k+1}}^{b_{k}}}}{\mathbf{p}_{b_{k+1}}^{w}} & 			\frac{\partial{\delta\theta_{b_{k+1}}^{b_{k}}}}{\mathbf{\theta}_{b_{k+1}}^{w}} & 
	\frac{\partial{\delta\theta_{b_{k+1}}^{b_{k}}}}{\mathbf{v}_{b_{k+1}}^{w}} & 			\frac{\partial{\delta\theta_{b_{k+1}}^{b_{k}}}}{\mathbf{b}_{a_{k+1}}}  &
	\frac{\partial{\delta\theta_{b_{k+1}}^{b_{k}}}}{\mathbf{b}_{w_{k+1}}} \\
	
	\frac{\partial{\delta\beta_{b_{k+1}}^{b_{k}}}}{\mathbf{p}_{b_{k}}^{w}} & 				\frac{\partial{\delta\beta_{b_{k+1}}^{b_{k}}}}{\mathbf{\theta}_{b_{k}}^{w}} & 
	\frac{\partial{\delta\beta_{b_{k+1}}^{b_{k}}}}{\mathbf{v}_{b_{k}}^{w}} & 				\frac{\partial{\delta\beta_{b_{k+1}}^{b_{k}}}}{\mathbf{b}_{a_{k}}}  &
	\frac{\partial{\delta\beta_{b_{k+1}}^{b_{k}}}}{\mathbf{b}_{w_{k}}}  &
	\frac{\partial{\delta\beta_{b_{k+1}}^{b_{k}}}}{\mathbf{p}_{b_{k+1}}^{w}} & 				\frac{\partial{\delta\theta_{b_{k+1}}^{b_{k}}}}{\mathbf{\theta}_{b_{k+1}}^{w}} & 
	\frac{\partial{\delta\beta_{b_{k+1}}^{b_{k}}}}{\mathbf{v}_{b_{k+1}}^{w}} & 				\frac{\partial{\delta\beta_{b_{k+1}}^{b_{k}}}}{\mathbf{b}_{a_{k+1}}}  &
	\frac{\partial{\delta\theta_{b_{k+1}}^{b_{k}}}}{\mathbf{b}_{w_{k+1}}} \\
	
	\frac{\partial{\delta{\mathbf{b}}_{{a}}}}{\mathbf{p}_{b_{k}}^{w}} & 					\frac{\partial{\delta{\mathbf{b}}_{{a}}}}{\mathbf{\theta}_{b_{k}}^{w}} & 
	\frac{\partial{\delta{\mathbf{b}}_{{a}}}}{\mathbf{v}_{b_{k}}^{w}} & 					\frac{\partial{\delta{\mathbf{b}}_{{a}}}}{\mathbf{b}_{a_{k}}}  &
	\frac{\partial{\delta{\mathbf{b}}_{{a}}}}{\mathbf{b}_{w_{k}}}  &
	\frac{\partial{\delta{\mathbf{b}}_{{a}}}}{\mathbf{p}_{b_{k+1}}^{w}} & 					\frac{\partial{\delta{\mathbf{b}}_{{a}}}}{\mathbf{\theta}_{b_{k+1}}^{w}} & 
	\frac{\partial{\delta{\mathbf{b}}_{{a}}}}{\mathbf{v}_{b_{k+1}}^{w}} & 					\frac{\partial{\delta{\mathbf{b}}_{{a}}}}{\mathbf{b}_{a_{k+1}}}  &
	\frac{\partial{\delta{\mathbf{b}}_{{a}}}}{\mathbf{b}_{w_{k+1}}} \\
	
	\frac{\partial{\delta{\mathbf{b}}_{{w}}}}{\mathbf{p}_{b_{k}}^{w}} & 					\frac{\partial{\delta{\mathbf{b}}_{{w}}}}{\mathbf{\theta}_{b_{k}}^{w}} & 
	\frac{\partial{\delta{\mathbf{b}}_{{w}}}}{\mathbf{v}_{b_{k}}^{w}} & 					\frac{\partial{\delta{\mathbf{b}}_{{w}}}}{\mathbf{b}_{a_{k}}}  &
	\frac{\partial{\delta{\mathbf{b}}_{{w}}}}{\mathbf{b}_{w_{k}}}  &
	\frac{\partial{\delta{\mathbf{b}}_{{w}}}}{\mathbf{p}_{b_{k+1}}^{w}} & 					\frac{\partial{\delta{\mathbf{b}}_{{w}}}}{\mathbf{\theta}_{b_{k+1}}^{w}} & 
	\frac{\partial{\delta{\mathbf{b}}_{{w}}}}{\mathbf{v}_{b_{k+1}}^{w}} & 					\frac{\partial{\delta{\mathbf{b}}_{{w}}}}{\mathbf{b}_{a_{k+1}}}  &
	\frac{\partial{\delta{\mathbf{b}}_{{w}}}}{\mathbf{b}_{w_{k+1}}} 
\end{bmatrix}
$$

 其中：
$$
\begin{align}
\frac{\partial{\delta\alpha_{b_{k+1}}^{b_{k}}}}{\mathbf{p}_{b_{k}}^{w}} &= \frac{\partial({\mathbf{R}_{w}^{b_k}(\mathbf{p}_{b_{k+1}}^{w} - \mathbf{p}_{b_{k}^{w}} - 						\mathbf{\upsilon}_{b_{k}^{w}} \Delta{t_k} + \frac{1}{2}\mathbf{g}^w 					\Delta{t_k}^2) -  \hat{\alpha}_{b_{k+1}}^{b_{k}}})}{\partial{\mathbf{p}_{b_{k}}^{w}}} \\
&= -\mathbf{R}_{w}^{b_k}

\end{align}
$$

$$
\begin{align}
	\frac{\partial{\delta\alpha_{b_{k+1}}^{b_{k}}}}{\mathbf{\theta}_{b_{k}}^{w}} &= 	\frac{\partial({\mathbf{R}_{w}^{b_k}(\mathbf{p}_{b_{k+1}^{w}} - \mathbf{p}_{b_{k}^{w}} - 						\mathbf{\upsilon}_{b_{k}^{w}} \Delta{t_k} + \frac{1}{2}\mathbf{g}^w 					\Delta{t_k}^2) -  \hat{\alpha}_{b_{k+1}}^{b_{k}}})}{\partial{\mathbf{\theta}_{b_{k}}^{w}}} \\
&= \lim_{\Delta {\mathbf{\theta}_{b_{k}}^{w}} \to 0}{\frac
{
\mathbf{R}_{b_k}^{w}exp([\Delta{\theta_{b_k}^{w}}]_{\times})^{T}(\mathbf{p}_{b_{k+1}^{w}} - \mathbf{p}_{b_{k}^{w}} - 						\mathbf{\upsilon}_{b_{k}^{w}} \Delta{t_k} + \frac{1}{2}\mathbf{g}^w 					\Delta{t_k}^2) -  \hat{\alpha}_{b_{k+1}}^{b_{k}}
}{\Delta {\mathbf{\theta}_{b_{k}}^{w}}}} \\

&= \lim_{\Delta {\mathbf{\theta}_{b_{k}}^{w}} \to 0}{\frac
{
\mathbf{R}_{b_k}^{w}exp([\Delta{\theta_{b_k}^{w}}]_{\times})^{T}(\mathbf{p}_{b_{k+1}^{w}} - \mathbf{p}_{b_{k}^{w}} - \mathbf{\upsilon}_{b_{k}^{w}} \Delta{t_k} + \frac{1}{2}\mathbf{g}^w \Delta{t_k}^2) -  
	(\mathbf{R}_{b_k}^{w})^{T} (\mathbf{p}_{b_{k+1}^{w}} - \mathbf{p}_{b_{k}^{w}} - \mathbf{\upsilon}_{b_{k}^{w}} \Delta{t_k} + \frac{1}{2}\mathbf{g}^w \Delta{t_k}^2)
}
{
	\Delta {\mathbf{\theta}_{b_{k}}^{w}}}
}


\end{align}
$$



#### 后端优化

##### 状态向量

##### 目标函数

$$
\min_{\small{\chi}}

\left \{
		{||r_{P} - H_{P}\chi||^{2} + 
		\sum_{k \in B}{|| r_{B}(\hat{z}_{b_{k+1}}^{b_k})||}_{p_{b_{k+1}}^{b_k}}^{2} + 
		\sum_{(l, j)\in{C}} ||r_C(\hat{z}_{l}^{c_j}), \chi)||_{P_{l}^{c_j}}^{2}}
\right \}
$$



#### IMU测量约束 

#### 视觉测量约束





## Part VI Summery





