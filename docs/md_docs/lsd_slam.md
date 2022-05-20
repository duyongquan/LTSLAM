# LSD-SLAM

## 1 LSD-SLAM algorithm

![image-20220520111527537](/home/quan/.config/Typora/typora-user-images/image-20220520111527537.png)

图像位姿跟踪:

>  跟踪部分主要任务是估计连续的图像对应的相机的位姿，它根据当前关键帧估计当前图像对应的相机位姿。相机位姿的初始值使用上一帧图像对应的相机位姿。	

深度估计:

>  首先根据估计得到的相机位姿的变化，判断是否使用该帧图像创建关键帧，当相机距离当前使用的关键帧对应的时刻相机的距离大于一定阈值，则使用该帧图像创建新的关键帧，并得到新的深度地图。如果距离过短，则将该帧图像的深度估计结果进行卡尔曼融合到当前使用的关键帧上。无论是否新建关键帧，都需要进行深度估计，并进行深度正则化，筛除离群点。

地图优化:

> 这个部分完成两个任务：位姿优化及点云地图优化功能。此处的位姿优化与第一部分?(跟踪)不同的是，跟踪部分是每帧图像的位姿估计，使用6自由度的变换矩阵se(3)来评估，用于评估深度，而第三(本)部分的位姿是基于关键帧之间的位姿估计，使用的是相似矩阵sim(3)来评估。
> 当关键帧成为参考帧，其对应的深度图不再被优化，则将该帧的深度图融合到全局地图。回环检测和尺度估计，对相近的两个关键帧进行相似变换矩阵评估，仍使用直接图像匹配法。

## 2 Error Function

$$
E({\xi_{ji}}) = \sum_{p\in \Omega_{D_i}} ||  \frac{r_{p}^2}{\sigma_{r_p}^{2}(p, \xi_{ji})} + \frac{r_{d}^2}{\sigma_{r_d}^{2}(p, \xi_{ji})}  ||_{\delta}
$$

with:
$$
\begin{aligned}
    r_{p}(p, \xi_{ji}) &= I_{i}(p) - I_{j}(w(p, D_i(p), \xi_{ji})) \\
    \sigma_{r_d}^{2}(p, \xi_{ji}) &= 2\sigma_{I}^{2} + (\frac{\partial{ r_{p}(p, \xi_{ji})}}{\partial{D_i(p)}})^2V_i(p)
\end{aligned}
$$
$∣∣ ∗ ∣∣_δ$ 是Huber范函数
$$
||r^2||_{\delta} =
\begin{cases}
	\frac{r^2}{2\delta} 		\quad \quad  \quad if|r| \le \delta \\
	|r| - \frac{\delta}{2}		\quad 	otherwise
\end{cases}
$$


### 2.1 深度的计算大概分为三个步骤

* 计算参考帧中的极线
* 确定极线上的最佳匹配
* 计算深度



![img](https://pic4.zhimg.com/v2-7ddd2b8de8ef9829ed9d77b65cc8f09f_b.jpg)

**几何误差**

----

极线的表达:
$$
L = \left\{ 
	l_0 + 
	\lambda 
		\left( 
		\begin{array} 
			cl_x \\ 
			l_y 
		\end{array}
	
		\right)|\lambda \in S
\right\}
$$
其中：

* $l_0$为深度为无穷大对应的点, 表示了极线的位置
* $(l_x, l_y)^T$ 为极线方向
* $S$为极线搜索范围



**光度误差**

----

在推导几何误差时，假设了等光度线方程没有误差，现在就来考虑这部分误差

在极线搜索时是要找到如下点：
$$
\lambda^{\star} =min (i_{ref} - I_{p}(\lambda))^2
$$
对 ![[公式]](https://www.zhihu.com/equation?tex=I_p%28%5Clambda%29) 泰勒展开：
$$
I_{p}(\lambda) = I_{p}(\lambda_{0}) + (\lambda) - \lambda_{0}) \ast g_p
$$

* ![[公式]](https://www.zhihu.com/equation?tex=g_p) 为图像梯度

所以可以代入求导等于0，便得到：

![[公式]](https://www.zhihu.com/equation?tex=%5Clambda+%5E+%7B+%2A+%7D+%28+I+%29+%3D+%5Clambda+_+%7B+0+%7D+%2B+%5Cleft%28+i+_+%7B+%5Ctext+%7B+ref+%7D+%7D+-+I+_+%7B+p+%7D+%5Cleft%28+%5Clambda+_+%7B+0+%7D+%5Cright%29+%5Cright%29+g+_+%7B+p+%7D+%5E+%7B+-+1+%7D%5C%5C)

这里考虑误差来源为参考帧与当前帧的光度误差，且认为其误差相同，所以：

![[公式]](https://www.zhihu.com/equation?tex=%5Csigma+_+%7B+%5Clambda+%28+I+%29+%7D+%5E+%7B+2+%7D+%3D+J+_+%7B+%5Clambda+%5E+%7B+%2A+%7D+%28+I+%29+%7D+%5Cleft%28+%5Cbegin%7Barray%7D+%7B+c+c+%7D+%7B+%5Csigma+_+%7B+i+%7D+%5E+%7B+2+%7D+%7D+%26+%7B+0+%7D+%5C%5C+%7B+0+%7D+%26+%7B+%5Csigma+_+%7B+i+%7D+%5E+%7B+2+%7D+%7D+%5Cend%7Barray%7D+%5Cright%29+J+_+%7B+%5Clambda+%5E+%7B+%2A+%7D+%28+I+%29+%7D+%3D+%5Cfrac+%7B+2+%5Csigma+_+%7B+i+%7D+%5E+%7B+2+%7D+%7D+%7B+g+_+%7B+p+%7D+%5E+%7B+2+%7D+%7D%5C%5C)

我们可以看到图像梯度越大带来的匹配误差也就越小，从下图中也可以理解：

![img](https://pic2.zhimg.com/v2-7a38290f5712f51fe23fab3a2dcac729_b.jpg)

其实几何误差与光度误差这两个来源分别影响的是以下方程的左右条线，也就是 ![[公式]](https://www.zhihu.com/equation?tex=l_0) 和 ![[公式]](https://www.zhihu.com/equation?tex=g_0)

![[公式]](https://www.zhihu.com/equation?tex=l+_+%7B+0+%7D+%2B+%5Clambda+%5E+%7B+%2A+%7D+%5Cleft%28+%5Cbegin%7Barray%7D+%7B+c+%7D+%7B+l+_+%7B+x+%7D+%7D+%5C%5C+%7B+l+_+%7B+y+%7D+%7D+%5Cend%7Barray%7D+%5Cright%29+%5Cstackrel+%7B+%21+%7D+%7B+%3D+%7D+g+_+%7B+0+%7D+%2B+%5Cgamma+%5Cleft%28+%5Cbegin%7Barray%7D+%7B+c+%7D+%7B+-+g+_+%7B+y+%7D+%7D+%5C%5C+%7B+g+_+%7B+x+%7D+%7D+%5Cend%7Barray%7D+%5Cright%29+%2C+%5Cquad+%5Cgamma+%5Cin+%5Cmathbb+%7B+R+%7D%5C%5C)

我们把两部分来源的误差可以看到一张图上理解：

![img](https://pic1.zhimg.com/v2-5c0392806d801e0af63fb34b3b4b5b20_b.jpg)

总得来说，几何来源误差取决于极线与梯度夹角，光度来源误差取决于极线上的像素梯度大小。



**逆深度误差**

----

在相机旋转较小时，逆深度误差与匹配误差成正比
$$
\sigma_{d, obs} = \alpha^2(\sigma_{\lambda(\xi,\pi)}^2 + \sigma_{\lambda(I)}^2)
$$
原因可从立体相机模型考虑:

![img](https://pic4.zhimg.com/v2-2175733e8f04a4a482956aa923c3a06b_b.jpg)

深度$z$表达式
$$
z = \frac{fb}{d} = \frac{fb}{u_l - u_r}
$$
前文讲述的匹配的误差也就是 ![[公式]](https://www.zhihu.com/equation?tex=d) 的误差，与深度成反比，与逆深度成正比。



**深度观测融合**

----

我们假设逆深度服从高斯分布，前文也已经论述深度需要多次估计优化收敛，也就需要将多次观测的逆深度值进行融合，这里用的就是卡尔曼滤波（按不确定度加权）：
$$
\mathcal N
\left (
\frac{\sigma_p^{2} d_o + \sigma_o^{2} d_p }{\sigma_p^{2} + \sigma_o^{2}}, 
\frac{\sigma_p^{2}  \sigma_o^{2}}{\sigma_p^{2} + \sigma_o^{2}}
\right)
$$

## 3 深度传播

将深度由一帧传递到下一帧时，忽略相机旋转：
$$
d_1(d_0) = (d_0^{-1} - t_z^{-1})^{-1}
$$
![[公式]](https://www.zhihu.com/equation?tex=t_z) 为相机沿光轴方向的位移。因此，可以计算新的不确定度：
$$
\sigma_{d_1}^2 = J_{d_{1}}\sigma_{d_{0}}^2J_{d_{1}}^{T} + \sigma_{p}^2 = 
(\frac{d_1}{d_0})^4 \sigma_{d_{0}}^2 + \sigma_{p}^2
$$
如果有两个点经过深度传播时传递到同一个像素点，则分两种情况处理：两者相近（在 ![[公式]](https://www.zhihu.com/equation?tex=2%5Csigma) 内）进行不确定度的加权融合；否则舍弃那个较远的。