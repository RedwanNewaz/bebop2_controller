import numpy as np
from py_traj_gen import PyTrajGen
from math import pi, sin, cos
import matplotlib.pyplot as plt

def getPath(numPoints, xScale, yScale, t0=0):
    for i in range(numPoints + 2):
        t = 2 * pi * (i + t0) / numPoints
        x = xScale * cos(t) * sin(t) # // You can adjust the scaling factor (2) for size
        y = yScale * sin(t)
        yield x, y

if __name__ == '__main__':
    xScale = 1.2
    yScale = 3.4
    numPoints = 100
    t0 = 26

    max_vel = 4
    max_acc = 2
    plannerType = "MinJerk"

    path = np.array([(x, y) for x, y in getPath(numPoints, xScale, yScale, t0)])
    print(path.shape)


    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    planner = PyTrajGen(max_vel, max_acc, plannerType)
    for x, y in path:
        planner.addWaypoint(x, y, 1.0)
    traj = planner.getTraj()
    start = 0
    plt.plot(path[:, 0], path[:, 1])
    # target, = plt.scatter(path[0, 0], path[0, 1], color='red', s=50)
    target, = plt.plot([path[0, 0]], [path[0, 1]], 'ro')
    for t, x, y, z in traj:

        target.set_data(x, y)
        delta = t - start + 1e-9
        start = t
        plt.pause(delta)
    plt.show()
