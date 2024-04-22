from math import pi, sin, cos


class Direction(int):
    clockWise = -1
    counterClockWise = 1

def getEightPath(numPoints, xScale, yScale, dir=1, t0=0):
    assert numPoints %  2 == 0
    for i in range(numPoints + 2):
        t = 2 * pi * (i + t0) / numPoints
        x = xScale * cos(t) * sin(t * dir) # // You can adjust the scaling factor (2) for size
        y = yScale * sin(t * dir)
        yield x, y