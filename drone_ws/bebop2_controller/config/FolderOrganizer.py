from pyTree.Tree import Tree as Tree
import os


def createDirs(root, dirName=""):
    if not root:
        return

    dirName += f"{root}" if root.isRoot() else f"/{root}"
    for child in root.getChildren():
        createDirs(child, dirName)
    if root.isBranch() and len(dirName):
        print(f"{dirName}")
        os.makedirs(dirName, exist_ok=True)


def getFolderHierarchy():
    root = Tree('.')
    model = Tree('model')
    kinematic = Tree('kinematic')
    dynamic = Tree('dynamic')

    controller = Tree('controller')
    PID = Tree('PID')
    LQR = Tree('LQR')
    interface = Tree('interface')

    perception = Tree('perception')

    sensor = Tree('sensor')
    landmark = Tree('landmark')
    mocap = Tree('mocap')
    dummy = Tree('dummy')

    filter = Tree('filter')

    LPF = Tree('LPF')
    EKF = Tree('EKF')
    PF = Tree('PF')

    navigation = Tree('navigation')
    trajectory = Tree('trajectory')
    waypoints = Tree('waypoints')
    ptp_move = Tree('ptp_move')

    launch = Tree('launch')

    conf = Tree('config')

    action = Tree('action')

    src = Tree('src')

    include = Tree('include')

    test = Tree('test')

    simulation = Tree('simulation')

    # add connections
    navigation.addChildren([trajectory, waypoints, ptp_move, interface])

    filter.addChildren([LPF, EKF, PF])

    sensor.addChildren([landmark, mocap, dummy])

    perception.addChildren([sensor, filter, interface])

    controller.addChildren([PID, LQR, interface])

    model.addChildren([kinematic, dynamic])

    root.addChildren([
        model, perception, controller, navigation, launch, conf, action, src,
        include, test, simulation
    ])
    return root


if __name__ == '__main__':

    root = getFolderHierarchy()
    root.prettyTree()
    createDirs(root)
