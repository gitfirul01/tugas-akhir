import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Generate some sample data
x = np.linspace(-5, 5, 100)
y = np.sin(x)
z = np.cos(x)

# Plot the data
ax.plot(x, y, z)

# Set the x-axis limits
ax.set_xlim(-5, 5)

# Show the plot
plt.show()
