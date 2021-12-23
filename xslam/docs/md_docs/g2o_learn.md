<center> <font color='green' size=12>G2O</font></center>

作者： 杜永全

邮箱：quandy2020@126.com

## 0 Introduction

g2o源码地址：

```shell
https://github.com/RainerKuemmerle/g2o.git
```

g2o的结构 :

<img src='./images/g2o/g2o_struct.png'>

如你所见，g2o项目中含有若干文件夹。刨开那些gitignore之类的零碎文件，主要有以下几个：

- EXTERNAL　　			三方库，有ceres, csparse, freeglut，可以选择性地编译；
- cmake_modules　　 给cmake用来寻找库的文件。我们用g2o时也会用它里头的东西，例如FindG2O.cmake
- doc　　　　　          文档。包括g2o自带的说明书（难度挺大的一个说明文档）。
- g2o　　　　　　      最重要的源代码都在这里！
- script　　　　           在android等其他系统编译用的脚本，由于我们在ubuntu下就没必要多讲了。

综上所述，最重要的就是g2o的源代码文件啦！所以我们要进一步展开看一看！



我们同样地介绍一下各文件夹的内容：

- apps　　　　一些应用程序。好用的g2o_viewer就在这里。其他还有一些不常用的命令行工具等。
- core　　　　核心组件，很重要！基本的顶点、边、图结构的定义，算法的定义，求解器接口的定义在这里。
- examples　　一些例程，可以参照着这里的东西来写。不过注释不太多。
- solvers　　　求解器的实现。主要来自choldmod, csparse。在使用g2o时要先选择其中一种。
- stuff　　　　对用户来讲可有可无的一些工具函数。
- types　　　   各种顶点和边，很重要！我们用户在构建图优化问题时，先要想好自己的顶点和边是否已经提供了定义。如果没有，要自己实现。如果有，就用g2o提供的即可。



​		就经验而言，solvers给人的感觉是大同小异，而 types 的选取，则是 g2o 用户主要关心的内容。然后 core 下面的内容，我们要争取弄的比较熟悉，才能确保使用中出现错误可以正确地应对。

　　那么，g2o最基本的类结构是怎么样的呢？我们如何来表达一个Graph，选择求解器呢？我们祭出一张图：

<img src='./images/g2o/g2o_class.png'>

一个是 SparseBlockMatrix ，用于计算稀疏的雅可比和海塞； 一个是用于计算迭代过程中最关键的一步 :
$$
H\Delta{x} = b
$$
这就需要一个线性方程的求解器。而这个求解器，可以从 PCG, CSparse, Choldmod 三者选一。

综上所述，在g2o中选择优化方法一共需要6个步骤：

> 1. 选择一个线性方程求解器，从 PCG, CSparse, Choldmod中选，实际则来自 g2o/solvers 文件夹中定义的东东。
> 2. 创建BlockSolver。并用上面定义的线性求解器初始化。
> 3. 创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化。
> 4. 创建终极大boss 稀疏优化器（SparseOptimizer），并用已定义求解器作为求解方法。
> 5. 定义图的顶点和边。并添加到SparseOptimizer中。
> 6. 设置优化参数，开始执行优化。

## 1 (Hyper)Graph-Embeddable Optimization Problems

最小二乘优化问题描述如下：
$$
\begin{align}
	F(x) &= 
		\sum_{k\in {C}}
			\underbrace
			{
				\mathbf{e}_{k}(x_k, z_k)^{T}\Omega_{k}\mathbf{e}_{k}(x_k, z_k)
			}_{F_{k}}
	\\
	x^{\star} &= argmin{F(\mathbf{x})}

\end{align}
$$

## 2 Least Squares Optimization

误差方程一阶泰勒展开：
$$
\begin{align}
	\breve{\mathbf{e}}(x_k + \Delta{x_k}) &= 
		\breve{\mathbf{e}}(x_k + \mathbf{\Delta{x}}) \\
		&\backsimeq \mathbf{e}_k + J_k\Delta{\mathbf{x}}
\end{align}
$$
where : $J_k$ Jacobian

so that, we obtatin :
$$
\begin{align}
F_k(\mathbf{x} + \Delta{\mathbf{x}})
	&= \breve{\mathbf{e}}(x_k + \Delta{x_k})^{T}\Omega_{k}\breve{\mathbf{e}}(x_k + \Delta{x_k}) \\
	
	&\backsimeq (\mathbf{e}_k + J_k\Delta{\mathbf{x}})^{T}\Omega_{k}(\mathbf{e}_k + J_k\Delta{\mathbf{x}})\\
	
	&= \underbrace{\mathbf{e}_k^{T}\Omega_{k}\mathbf{e}_k}_{c_k}+ 
	2\underbrace{\mathbf{e}_k^{T}\Omega_{k}J_k^{T}}_{\mathbf{b}_k} \Delta{\mathbf{x}} +
	\Delta{\mathbf{x}}^{T}\underbrace{J_k^{T} \Omega_{k}J_k}_{H_k}\Delta{\mathbf{x}} \\
		
   &= c_k + 2\mathbf{b}_k\Delta{\mathbf{x}} +
   		\Delta{\mathbf{x}}^{T}H_k\Delta{\mathbf{x}}

	
\end{align}
$$
It can be minimized in $\Delta{\mathbf{x}}$ by solving the linear system :
$$
H\Delta{\mathbf{x^{\star}}} = -\mathbf{b}
$$

## 3 Considerations about the Structure of the Linearized System



## 4 Least Squares on Manifold



## 5 Robust Least Squares

least squares optimization can be robustified, rror terms:
$$
F_k = \mathbf{e}_k^{T}\Omega_k\mathbf{e}_k = \rho_{2}
\left( 
	\sqrt{\mathbf{e}_k^{T}\Omega_k\mathbf{e}_k}
\right)
\quad with \quad 
\rho_{2}(x) := x^2
$$
Huber cost function $ρ_H$:
$$
\rho_{H}(x): =
\begin{cases}
	x^2 	\quad \quad \quad \quad\text {if |x| < b} \\
	2b|x| - b^2 \quad \text{else}
\end{cases}
$$
Then, $e_k$ is replaced by a weighted version :
$$
(w_k\mathbf{e}_k)^{T}\Omega_{k}w_k(\mathbf{e}_k) = \rho_{H}
\left(
	\sqrt{\mathbf{e}_k^{T}\Omega_k\mathbf{e}_k}
\right)
$$
Here, the weights wk are calculated as follows:
$$
w_k = \frac{\sqrt{\rho_{H}(||\mathbf{e}_k||_{\Omega})}}{||\mathbf{e}_k||_{\Omega}}

\quad with  \quad ||\mathbf{e}_k||_{\Omega}:= \sqrt{\mathbf{e}_k^{T}\Omega_k\mathbf{e}_k}
$$

## 6 Library Overview

### 6.1 Representation of an Optimization Problem

* BaseVertex
* BaseBinaryEdge
* BaseMultiEdge

### 6.2 Construction and Representation of the Linearized Problem

* Initialization
* Compute error
* Linearizing the system
* Building the system
* Updating Levenberg-Marquardt
* Solvers
  * PCG
  * G-N
  * L-M

## 7 g2o Tools

g2o 带有两个工具，可以处理存储在文件中的数据。 数据可以从文件中加载并在处理后再次存储。 下面我们将简要介绍这些工具，即命令行界面和图形用户界面。

###  7.1 g2o Command Line Interface

g2o 是 g2o 中包含的命令行界面。 它允许优化存储在文件中的图形并将结果保存回文件。 这允许优化问题的快速原型设计，因为它只需要实现新类型或求解器。 g2o 发行版包括一个数据文件夹，其中包含一些可以应用 g2o 的数据文件。

### 7.2 g2o Viewer

图 3 中描绘的图形用户界面允许可视化优化问题。 此外，可以控制算法的各种参数。

### 7.3 g2o incremental

g2o 包含一个实验二进制文件，用于以增量方式执行优化，即在插入一个或多个节点及其测量值后进行优化。 在这种情况下，g2o 对 Hessian 矩阵执行 ranke 更新以更新线性系统。 有关其他信息，请参阅 g2o_incremental 子文件夹中的自述文件。

Example for the Manhattan3500 dataset :

```shell
g2o_incremental -i manhattanOlson3500.g2o
```



### 7.4 Plug-in Architecture



## 8 2D SLAM: An Example



## 9 G2O Example

所有代码均为官方自带demo, 本文实现的demo的git地址 :

```shell
git clone https://gitee.com/quanduyong/slam_tutorial.git
```

### 9.1 circle_fit

* Linear least squares solution :

circle function:
$$
(x - a)^2 + (y - b)^2 = r^2
$$
Let$ (a, b)$ be the center of the circle and $r $the radius of the circle :
$$
\begin{bmatrix}
-2x &
-2y &
1
\end{bmatrix}

\begin{bmatrix}
a \\
b \\ c
\end{bmatrix}
=
-x^2 - y^2
$$
where : $c = a^2 + b^2 - r^2$

rewrite circle function ：
$$
\underbrace
{
	 \begin{bmatrix}
    -2x &
    -2y &
    1
    \end{bmatrix}
}_\mathbf{A} 
  
\underbrace
{
    \begin{bmatrix}
    a \\
    b \\ c
    \end{bmatrix}
}_\mathbf{x}
=
\underbrace
{
	-x^2 - y^2
}_\mathbf{b}
$$


compute the normal equation to obtain a solution for (a b c)

* Iterative least squares solution

```c++
Eigen::MatrixXd A(numPoints, 3);
Eigen::VectorXd b(numPoints);
for (int i = 0; i < numPoints; ++i) 
{
    A(i, 0) = -2*points[i].x();
    A(i, 1) = -2*points[i].y();
    A(i, 2) = 1;
    b(i) = -pow(points[i].x(), 2) - pow(points[i].y(), 2);
}
Eigen::Vector3d solution = (A.transpose()*A).ldlt().solve(A.transpose() * b);
```

circle极坐标的方程表示:
$$
\begin{cases}
	x = x_0 + r\cos{\theta} \\
	y = y_0 + r\sin{\theta}
\end{cases}
$$
where :  $r$ 半径; $(x_0, y_0)$圆心

均匀分布：

```c++
/**
 * sample a number from a uniform distribution
 */
static number_t uniformRand(number_t lowerBndr, number_t upperBndr)
{
	return lowerBndr + ((number_t) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}
```

高斯分布：

```c++
/**
 * Gaussian random with a mean and standard deviation. Uses the
 * Polar method of Marsaglia.
 */
static number_t gaussRand(number_t mean, number_t sigma)
{
    number_t x, y, r2;
    do {
        x = -1.0 + 2.0 * uniformRand(0.0, 1.0);
        y = -1.0 + 2.0 * uniformRand(0.0, 1.0);
        r2 = x * x + y * y;
    } while (r2 > 1.0 || r2 == 0.0);
    return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}
```

---

Vertex类型VertexCircle:

```c++
/**
 * A circle located at x,y with radius r
 */
class VertexCircle : public ::g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCircle(){}

    virtual bool read(std::istream& /*is*/);
    virtual bool write(std::ostream& /*os*/) const;

    virtual void setToOriginImpl();
    virtual void oplusImpl(const double* update);
};
```

```c++
bool VertexCircle::read(std::istream& /*is*/)
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

bool VertexCircle::write(std::ostream& /*os*/) const
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

void VertexCircle::setToOriginImpl()
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
}

void VertexCircle::oplusImpl(const double* update)
{
    Eigen::Vector3d::ConstMapType v(update);
    _estimate += v;
}
```

---

Edge类型EdgePointOnCircle：

```c++
bool EdgePointOnCircle::read(std::istream& /*is*/)
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

bool EdgePointOnCircle::write(std::ostream& /*os*/) const
{
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
}

void EdgePointOnCircle::computeError()
{
    const VertexCircle* circle = static_cast<const VertexCircle*>(vertex(0));

    const Eigen::Vector2d& center = circle->estimate().head<2>();
    const double& radius = circle->estimate()(2);

    _error(0) = (measurement() - center).norm() - radius;
}
```

设置真值 : $r = 2$ , center $(4, 2)$

误差项：
$$
r = (x - a)^2 + (y - b)^2-  r^2
$$

```c++
_error(0) = (measurement() - center).norm() - radius; 
```

加入高斯噪声数据求得： $r = 2.00662$ , center $(4.01514 1.99241)$

---



### 9.2 curve_fit

曲线方程：
$$
y = \exp(ax^2 + bx + c) + w
$$
where :

* $a, b, c$为曲线参数
* $w$为高斯噪声

Vertex类型:

```c++
// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex : public ::g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 重置
    virtual void setToOriginImpl() override {
        _estimate << 0, 0, 0;
    }

    // 更新
    virtual void oplusImpl(const double *update) override {
        _estimate += Eigen::Vector3d(update);
    }

    // 存盘和读盘：留空
    virtual bool read(std::istream &in) {}
    virtual bool write(std::ostream &out) const {}
};
```

Edge类型：

```c++
// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge : public ::g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    // 计算曲线模型误差
    virtual void computeError() override
    {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
    }

    // 计算雅可比矩阵
    virtual void linearizeOplus() override
    {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
        _jacobianOplusXi[0] = -_x * _x * y;
        _jacobianOplusXi[1] = -_x * y;
        _jacobianOplusXi[2] = -y;
    }

    virtual bool read(std::istream &in) {}
    virtual bool write(std::ostream &out) const {}
public:
    double _x;  // x 值， y 值为 _measurement
};
```

### 9.3 sphere



### 9.4 simple_optimize



### 9.5 ba



### 9.6 bal



### 9.7 ba_anchored_inverse_depth



### 9.8 data_convert



### 9.9 icp



### 9.10 slam2d



### 9.11 tutorial_slam2d



### 9.12 line_slam



### 9.13 plane_slam



### 9.14 interactive_slam



### 9.15 calibration_odom_laser



## 10 参考

1 [[深入理解图优化与g2o：g2o篇](https://www.cnblogs.com/gaoxiang12/p/5304272.html)](https://www.cnblogs.com/gaoxiang12/p/5304272.html)

2 [g2o: A general Framework for (Hyper) Graph Optimization](https://github.com/RainerKuemmerle/g2o/blob/master/doc/g2o.pdf)

