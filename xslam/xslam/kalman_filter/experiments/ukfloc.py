# -*- coding: utf-8 -*-
"""
Created on Mon Jun  1 18:13:23 2015

@author: rlabbe
"""

from filterpy.common import plot_covariance_ellipse
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints, JulierSigmaPoints
from math import tan, sin, cos, sqrt, atan2, radians, sqrt
import matplotlib.pyplot as plt
from numpy import array
import numpy as np
from numpy.random import randn, seed



def normalize_angle(x):
    x = x % (2 * np.pi)    # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi]
        x -= 2 * np.pi
    return x


def residual_h(aa, bb):
    y = aa - bb
    y[1] = normalize_angle(y[1])
    return y


def residual_x(a, b):
    y = a - b
    y[2] = normalize_angle(y[2])
    return y


def move(x, u, dt, wheelbase):
    h = x[2]
    v = u[0]
    steering_angle = u[1]

    dist = v*dt

    if abs(steering_angle) > 0.001:
        b = dist / wheelbase * tan(steering_angle)
        r = wheelbase / tan(steering_angle) # radius

        sinh = sin(h)
        sinhb = sin(h + b)
        cosh = cos(h)
        coshb = cos(h + b)

        return x + array([-r*sinh + r*sinhb, r*cosh - r*coshb, b])
    else:
        return x + array([dist*cos(h), dist*sin(h), 0])


def state_mean(sigmas, Wm):
    x = np.zeros(3)
    sum_sin, sum_cos = 0., 0.

    for i in range(len(sigmas)):
        s = sigmas[i]
        x[0] += s[0] * Wm[i]
        x[1] += s[1] * Wm[i]
        sum_sin += sin(s[2])*Wm[i]
        sum_cos += cos(s[2])*Wm[i]

    x[2] = atan2(sum_sin, sum_cos)
    return x


def z_mean(sigmas, Wm):
    x = np.zeros(2)
    sum_sin, sum_cos = 0., 0.

    for i in range(len(sigmas)):
        s = sigmas[i]
        x[0] += s[0] * Wm[i]
        sum_sin += sin(s[1])*Wm[i]
        sum_cos += cos(s[1])*Wm[i]

    x[1] = atan2(sum_sin, sum_cos)
    return x


sigma_r = .3
sigma_h =  .01#radians(.5)#np.radians(1)
#sigma_steer =  radians(10)
dt = 0.1
wheelbase = 0.5

m = array([[5., 10], [10, 5], [15, 15], [20, 5], [0, 30], [50, 30], [40, 10]])
#m = array([[5, 10], [10, 5], [15, 15], [20, 5],[5,5], [8, 8.4]])#, [0, 30], [50, 30], [40, 10]])
m = array([[5, 10], [10, 5]])#, [0, 30], [50, 30], [40, 10]])


def fx(x, dt, u):
    return move(x, u, dt, wheelbase)


def Hx(x, landmark):
    """ takes a state variable and returns the measurement that would
    correspond to that state.
    """
    px, py = landmark
    dist = sqrt((px - x[0])**2 + (py - x[1])**2)
    angle = atan2(py - x[1], px - x[0])
    return array([dist, normalize_angle(angle - x[2])])


points = MerweScaledSigmaPoints(n=3, alpha=.1, beta=2, kappa=0)
#points = JulierSigmaPoints(n=3,  kappa=3)
ukf= UKF(dim_x=3, dim_z=2, fx=fx, hx=Hx, dt=dt, points=points,
         x_mean_fn=state_mean, z_mean_fn=z_mean,
         residual_x=residual_x, residual_z=residual_h)
ukf.x = array([2, 6, .3])
ukf.P = np.diag([.1, .1, .2])
ukf.R = np.diag([sigma_r**2, sigma_h**2])
ukf.Q = None#np.eye(3)*.00000


u = array([1.1, 0.])

xp = ukf.x.copy()


plt.cla()
plt.scatter(m[:, 0], m[:, 1])

cmds = [[v, .0] for v in np.linspace(0.001, 1.1, 30)]
cmds.extend([cmds[-1]]*50)

v = cmds[-1][0]
cmds.extend([[v, a] for a in np.linspace(0, np.radians(2), 15)])
cmds.extend([cmds[-1]]*100)

cmds.extend([[v, a] for a in np.linspace(np.radians(2), -np.radians(2), 15)])
cmds.extend([cmds[-1]]*200)

cmds.extend([[v, a] for a in np.linspace(-np.radians(2), 0, 15)])
cmds.extend([cmds[-1]]*150)

#cmds.extend([[v, a] for a in np.linspace(0, -np.radians(1), 25)])



seed(12)
cmds = np.array(cmds)

cindex = 0
u = cmds[0]

track = []

while cindex < len(cmds):
    u = cmds[cindex]
    xp = move(xp, u, dt, wheelbase) # simulate robot
    track.append(xp)

    ukf.predict(fx_args=u)

    if cindex % 20 == 30:
        plot_covariance_ellipse((ukf.x[0], ukf.x[1]), ukf.P[0:2, 0:2], std=18,
                                facecolor='b', alpha=0.58)

    #print(cindex, ukf.P.diagonal())
    print(xp)
    for lmark in m:

        d = sqrt((lmark[0] - xp[0])**2 + (lmark[1] - xp[1])**2) + randn()*sigma_r
        bearing = atan2(lmark[1] - xp[1], lmark[0] - xp[0])
        a = normalize_angle(bearing - xp[2] + randn()*sigma_h)
        z = np.array([d, a])

        if cindex % 20 == 0:
            plt.plot([lmark[0], lmark[0] - d*cos(a+xp[2])], [lmark[1], lmark[1]-d*sin(a+xp[2])], color='r')

        ukf.update(z, hx_args=(lmark,))
        print(ukf.P)
    print()

    if cindex % 20 == 0:
        plot_covariance_ellipse((ukf.x[0], ukf.x[1]), ukf.P[0:2, 0:2], std=15,
                                facecolor='g', alpha=0.99)
    cindex += 1


track = np.array(track)
plt.plot(track[:, 0], track[:,1], color='k')

plt.axis('equal')
plt.title("UKF Robot localization")
plt.show()
print(ukf.P.diagonal())
