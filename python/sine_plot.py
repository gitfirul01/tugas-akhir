import numpy as np
import matplotlib.pyplot as plt

# Turn on interactive mode
plt.ion()

# Create a figure and axis
fig, ax = plt.subplots()

# Plot some initial data
x = np.linspace(0, 10, 100)
y = np.sin(x)
line, = ax.plot(x, y)

# Update the plot in a loop
for i in range(100):
    # Update the data
    y = np.sin(x + i / 10)
    line.set_ydata(y)
    
    # Redraw the plot
    fig.canvas.draw()
    fig.canvas.flush_events()
