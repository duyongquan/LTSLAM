import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0, 20)
y = 0.5 * (10 - x) ** 2

plt.plot(x, y)
plt.text(8, 40, r"$\frac{1}{2}(10 - x)^2$", fontsize=20)
plt.xlabel('x')
plt.ylabel('y')
plt.show()