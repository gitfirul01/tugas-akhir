import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# initial value
x = 0
y = 0
z = 0

# turn on interactive plot
plt.ion()

# create a 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# set figure configuration
ax.set_title('3D Simulation')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(0, 50)
ax.set_ylim(0, 50)
ax.set_zlim(0, 50)

# plot the data in 3D
plot = ax.scatter(x, y, z)

while True:
    x+=1
    y+=1
    z+=1
    
    # redraw the plot
    plot.remove()
    plot = ax.scatter(x, y, z)
    # show the plot
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.001)
    