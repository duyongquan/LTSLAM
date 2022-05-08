import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def plot_age_salary():
    age = [25, 27, 29, 31, 33, 35, 40]
    salary = [10000, 15000, 25000, 30000, 35000, 38000, 50000]

    plt.scatter(age,salary)
    plt.show()


# fx的函数值
# f = x^2
def univariate_function(x):
    return x**2


#定义梯度下降算法
def gradient_descent_univariate_function():
    times = 100 # 迭代次数
    alpha = 0.1 # 学习率
    x =10# 设定x的初始值
    x_axis = np.linspace(-10, 10) #设定x轴的坐标系
    fig = plt.figure(1,figsize=(5,5)) #设定画布大小
    ax = fig.add_subplot(1,1,1) #设定画布内只有一个图
    ax.set_xlabel('X', fontsize=14)
    ax.set_ylabel('Y', fontsize=14)
    ax.plot(x_axis,univariate_function(x_axis)) #作图
    
    for i in range(times):
        x1 = x          
        y1= univariate_function(x)  
        print("第%d次迭代：x=%f，y=%f" % (i + 1, x, y1))
        x = x - alpha * 2 * x
        y = univariate_function(x)
        ax.plot([x1,x], [y1,y], 'ko', lw=1, ls='-', color='coral')
    plt.show()


#求fx的函数值
def multivariate_function(x, y):
    return (x - 10) ** 2 + (y - 10) ** 2


def gradient_descent_multivariate_function():
    times = 200 # 迭代次数
    alpha = 0.05 # 学习率
    x = 20 # x的初始值
    y = 20 # y的初始值

    fig = Axes3D(plt.figure()) # 将画布设置为3D
    axis_x = np.linspace(0, 20, 100)#设置X轴取值范围
    axis_y = np.linspace(0, 20, 100)#设置Y轴取值范围
    axis_x, axis_y = np.meshgrid(axis_x, axis_y) #将数据转化为网格数据
    z = multivariate_function(axis_x,axis_y)#计算Z轴数值
    fig.set_xlabel('X', fontsize=14)
    fig.set_ylabel('Y', fontsize=14)
    fig.set_zlabel('Z', fontsize=14)
    fig.view_init(elev=60,azim=300)#设置3D图的俯视角度，方便查看梯度下降曲线
    fig.plot_surface(axis_x, axis_y, z, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow')) #作出底图
    
    for i in range(times):
        x1 = x        
        y1 = y         
        f1 = multivariate_function(x, y)  
        print("第%d次迭代：x=%f，y=%f，fxy=%f" % (i + 1, x, y, f1))
        x = x - alpha * 2 * (x - 10)
        y = y - alpha * 2 * (y - 10)
        f = multivariate_function(x, y)
        fig.plot([x1, x], [y1, y], [f1, f], 'r-', lw=2, ls='-')
    plt.show()


if __name__ == "__main__":
    # plot_age_salary()
    # gradient_descent_univariate_function()
    gradient_descent_multivariate_function()
    