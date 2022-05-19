import numpy as np
import holoviews as hv
hv.extension('plotly')


def conjugate_gradient(A, b):
    if (is_pos_def(A) == False) | (A != A.T).any():
        raise ValueError('Matrix A needs to be symmetric positive definite (SPD)')
    r = b 
    k = 0
    x = np.zeros(A.shape[-1])
    while LA.norm(r) > 1e-10 :
        if k == 0:
            p = r
        else: 
            gamma = - (p @ A @ r)/(p @ A @ p)
            p = r + gamma * p
        alpha = (p @ r) / (p @ A @ p)
        x = x + alpha * p
        r = r - alpha * (A @ p)
        k =+ 1
    return x

def conjugate_gradient_store_result(A, b):
    if (is_pos_def(A) == False) | (A != A.T).any():
        raise ValueError('Matrix A needs to be symmetric positive definite (SPD)')
    r = b 
    k = 0
    x = np.zeros(A.shape[-1])
    x_steps = [x]
    y_steps = [0.5 * x @ A @ x - x @ b]
    while LA.norm(r) > 1e-10 :
        if k == 0:
            p = r
        else: 
            gamma = - (p @ A @ r)/(p @ A @ p)
            p = r + gamma * p
        alpha = (p @ r) / (p @ A @ p)
        x = x + alpha * p
        r = r - alpha * (A @ p)
        k =+ 1
        x_steps.append(x)
        y_steps.append(0.5 * x @ A @ x - x @ b)

    return x, x_steps, y_steps

def viz_descent(x_steps, y_steps):
    size = 50
    x1s = np.linspace(-6, 6, size)
    x2s = np.linspace(-6, 6, size)
    x1, x2  = np.meshgrid(x1s, x2s)
    Z = np.zeros((size, size))
    for i in range(size):
        for j in range(size):
            x = np.array([x1[i,j], x2[i,j]])
            Z[i,j] = 0.5 * x @ A @ x - x @ b
    surface = hv.Surface((x1s, x2s, Z)).opts(colorbar=True, width=700, height=700, cmap='Turbo',  alpha=.75)
    points= np.concatenate([np.stack(x_steps), np.array(y_steps)[:, np.newaxis]], axis=1)
    path = hv.Path3D(points).opts(width=700, height=700, color='black', line_width=1)
    return surface * path

# visualize steepest descent method
_, x_steps, y_steps = steepest_descent_store_result(A, b, x0)
viz_descent(x_steps, y_steps)