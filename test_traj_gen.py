import numpy as np
from py_traj_gen import PyTrajGen
import matplotlib.pyplot as plt
from GeomPaths.Eight import getEightPath, Direction
from threading import Thread
from queue import Queue
from time import sleep

plt.gcf().canvas.mpl_connect(
    'key_release_event',
    lambda event: [exit(0) if event.key == 'escape' else None])
class VizTrajectory:
    count = 0
    def __init__(self, path, ax,   waitQueue):
        self.ax = ax
        self.waitQueue = waitQueue
        if self.count == 0:
            self.ax.plot(path[:, 0], path[:, 1])
            self.count += 1
    def __call__(self, *args, **kwargs):
        start = 0
        for i, (t, x, y, z) in enumerate(args[0]):
            if i == 0:
                target, = self.ax.plot(x, y, 'ro')
            else:
                target.set_data(x, y)
            delta = t - start + 1e-9
            start = t
            self.waitQueue.put(delta)
            sleep(delta)



def getTraj(dirc, t0):
    path = np.array([(x, y) for x, y in getEightPath(numPoints, xScale, yScale, dirc, t0)])
    planner = PyTrajGen(max_vel, max_acc, plannerType)
    for x, y in path:
        planner.addWaypoint(x, y, 1.0)
    traj = planner.getTraj()
    return path, traj


def analyzeTrajs():
    Nmin = min(len(traj1), len(traj2))
    Nmax = max(len(traj1), len(traj2))

    print("point diff {}, min points {}".format(Nmax - Nmin, Nmin))
    minDist = 1000.0
    for i in range(Nmin):
        data1 = np.array(traj1[i][1:3])
        data2 = np.array(traj2[i][1:3])
        # print(data1)
        minDist = min(minDist, np.linalg.norm(data1 - data2))
    print(f"minimum distance {minDist:.3f}")
def showAnimation():
    ax = plt.gca()
    waitQueue, waitQueue2 = Queue(), Queue()
    viz1 = VizTrajectory(path1, ax, waitQueue)
    viz2 = VizTrajectory(path2, ax, waitQueue2)


    thread1 = Thread(target=viz1, args=(traj1,))
    thread2 = Thread(target=viz2, args=(traj2,))

    thread1.start()
    thread2.start()

    while thread1.is_alive() or thread2.is_alive():
        if not waitQueue.empty():
            delta = waitQueue.get()
            plt.pause(delta)



if __name__ == '__main__':
    xScale = 1.2
    yScale = 3.4
    numPoints = 100
    t0 = 26 + 5

    max_vel = 4
    max_acc = 2
    plannerType = "MinSnap"
    t1 = 70

    path1, traj1 = getTraj(Direction.counterClockWise, t0)
    path2, traj2 = getTraj(Direction.counterClockWise, t1)
    analyzeTrajs()
    showAnimation()








    # start = 0
    #
    # # target, = plt.scatter(path[0, 0], path[0, 1], color='red', s=50)
    # target, = plt.plot([path[0, 0]], [path[0, 1]], 'ro')
    # for t, x, y, z in traj:
    #
    #     target.set_data(x, y)
    #     delta = t - start + 1e-9
    #     start = t
    #     plt.pause(delta)
    # plt.show()
