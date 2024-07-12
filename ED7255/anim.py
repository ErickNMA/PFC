import matplotlib.pyplot as plt
import numpy as np

import matplotlib.animation as animation

fig, ax = plt.subplots()

x = np.arange(0, 2*np.pi, 0.01)
line, = ax.plot(x, np.sin(x))


def animate(i):
    line.set_ydata(np.sin(x + i))  # update the data.
    return line,


ani = animation.FuncAnimation(
    fig, animate, interval=20, frames=50, blit=True, cache_frame_data=False, repeat=False)

plt.show()