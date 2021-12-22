import matplotlib.pyplot as plt
import numpy as np
import random


def GenerateCurveData(noise=False):
    m = 0.3
    c = 0.1
    x = np.linspace(0, 5, 100)
    y = np.exp(m * x + c)
    if noise==True:
        y += np.random.random(100)*0.1

    print(x)

    print(y)
    return [x, y]

def PlotGroundTrueCurve(x, y):
    plt.plot(x,y, 'r')
    # plt.show()

def PlotNoiseCurve(x, y):
    plt.scatter(x, y)
    plt.show()

if __name__=='__main__':
    # Ground True points
    [x, y] = GenerateCurveData()
    PlotGroundTrueCurve(x, y)

    # Noise points
    [x_noise, y_noise] = GenerateCurveData(True)
    PlotNoiseCurve(x_noise, y_noise)