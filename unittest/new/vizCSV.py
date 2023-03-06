import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys

# tag = 2 origin (5.000000 0.000000 0.950000)
# tag = 4 origin (5.000000 -2.050000 0.950000)
# tag = 7 origin (5.000000 -1.130000 0.950000)

def modifiedData0(csvfile):
    df = pd.read_csv(csvfile)
    df2 = df[df[" yaw"] == 2]
    df7 = df[df[" yaw"] == 7]
    df4 = df[df[" yaw"] == 4]
    legends = []
    offsetX, offsetY = -1.6, -1.1
    for i, d in zip([2, 4, 7], [df2, df4, df7]):
        legends.append(i)
        if(i == 4):
            X, Y = d["x"], d[" y"]
            arr = np.column_stack((X, Y))
            arr[:, 0] = arr[:, 0] + offsetX
            arr[:, 1] = arr[:, 1]  + 0.75 + offsetY

            plt.scatter(arr[:, 0], arr[:, 1], s=15)
        elif (i==2):
            X, Y = d["x"], d[" y"]
            arr = np.column_stack((X, Y))
            arr[:, 0] = 7.45 - arr[:, 0] + offsetX
            arr[:, 1] = 2.3 - arr[:, 1] + offsetY
            # arr[:, 1] = 0.5 - arr[:, 1]
            plt.scatter(arr[:, 0], arr[:, 1], s=15)
        else:
            X, Y = d["x"], d[" y"]
            arr = np.column_stack((X, Y))
            arr[:, 0] +=   offsetX
            arr[:, 1] +=  offsetY
            plt.scatter(arr[:, 0], arr[:, 1], s=15)
    plt.legend(legends)
    # plt.axis([0, 3, 0, 4.5])
    plt.show()


transform = {4: (-0.605, 2.2575, 0.85), 2: (-1.2, 0.85, -1.0), 7:(-0.84, 1.12, -0.9)}

transform2 ={
    2: [[1.0, 0, 0, -1.2], [0, -1.0, 0, 0.85], [0.0, 0, 1.0, 0.95], [0, 0, 0, 1]],
    4: [[0.85, 0, 0, -0.605], [0, 0.85, 0, 2.2575], [0.0, 0, 0.85, 0.95], [0, 0, 0, 1]],
    7: [[0.9, 0, 0, -0.84], [0, -0.9, 0, 1.12], [0.0, 0, 0.9, 0.95], [0, 0, 0, 1]]
}


def modifiedData1(csvfile):
    df = pd.read_csv(csvfile)
    df2 = df[df[" yaw"] == 2]
    df7 = df[df[" yaw"] == 7]
    df4 = df[df[" yaw"] == 4]
    legends = []

    for i, d in zip([2, 4, 7], [df2, df4, df7]):
        legends.append(i)
        X, Y = d["x"], d[" y"]
        transform[i][-1] = abs(transform[i][-1])
        X = (X * transform[i][-1]) + transform[i][0]

        Y = (Y * transform[i][-1]) + transform[i][1] if i == 4 else transform[i][1] -  (Y * transform[i][-1])
        plt.scatter(X, Y, s=15)
    plt.show()


def modifiedData2(csvfile):
    df = pd.read_csv(csvfile)
    df2 = df[df[" yaw"] == 2]
    df7 = df[df[" yaw"] == 7]
    df4 = df[df[" yaw"] == 4]
    legends = []

    for i, d in zip([2, 4, 7], [df2, df4, df7]):
        legends.append(i)
        X, Y = d["x"], d[" y"]
        trans = np.array(transform2[i])
        R = trans[:2, :2]
        O = trans[:2, -1]

        # R = np.eye(2)
        # O = np.array(transform[i][:-1])
        # R = R * transform[i][-1]
        # R[0] = abs(R[0])
        print(R, O)
        P = np.column_stack((X, Y))
        P = O + P @ R
        plt.scatter(P[:, 0], P[:, 1], s=15)
    plt.show()

def showData0(csvfile):
    df = pd.read_csv(csvfile)
    df2 = df[df[" yaw"] == 2]
    df7 = df[df[" yaw"] == 7]
    df4 = df[df[" yaw"] == 4]
    legends = []

    for i, d in zip([2, 4, 7], [df2, df4, df7]):
        legends.append(i)
        X, Y = d["x"], d[" y"]
        plt.scatter(X, Y, s=15)

    plt.legend(legends)
    plt.show()
def showData(csvfile):
    df = pd.read_csv(csvfile)

    X, Y = df["x"], df[" y"]
    plt.scatter(X, Y, s=15)
    plt.show()

if __name__ == '__main__':
    csvfile = sys.argv[1]
    showData0(csvfile)
