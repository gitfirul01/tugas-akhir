import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Generate some sample 3D data
x = np.linspace(-1, 1, 100)
y = np.linspace(-1, 1, 100)
X, Y = np.meshgrid(x, y)
Z = np.sin(np.sqrt(X**2 + Y**2))

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plot = ax.plot_surface(X, Y, Z)

# Update the plot
for i in range(100):
    # Modify the data
    Z = np.sin(np.sqrt((X+i/10)**2 + Y**2))

    # Redraw the plot
    plot.remove()
    plot = ax.plot_surface(X, Y, Z)
    plt.draw()
    plt.pause(0.001)

# Show the plot
plt.show()
