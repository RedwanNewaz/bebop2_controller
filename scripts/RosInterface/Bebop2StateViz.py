from typing import Any
import rospy 
import math
import numpy as np 
from copy import deepcopy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy.spatial.transform import Rotation as Rot
from RobotModel import  convert_spherical_to_cartesian

def rot_mat_2d(angle):
    """
    Create 2D rotation matrix from an angle

    Parameters
    ----------
    angle :

    Returns
    -------
    A 2D rotation matrix

    Examples
    --------
    >>> angle_mod(-4.0)


    """
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]


class Bebop2StateViz:
    def __init__(self, landmarks) -> None:
        self.landmarks = landmarks
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_landmark)
        self.marker_pub = rospy.Publisher('/bebop2/goal', Marker, queue_size=1)
        self.position = [0, 0, 0]
        self.orientation = [0, 0, 0, 1]


    def __call__(self, state):
        self.position = state[:3]
        self.orientation = quaternion_from_euler(0, 0, state[3] + math.pi/2)

        bebop_marker = self.populate_marker()
        bebop_marker.pose.orientation.x = self.orientation[0]  # Set the orientation of the marker
        bebop_marker.pose.orientation.y = self.orientation[1]
        bebop_marker.pose.orientation.z = self.orientation[2]
        bebop_marker.pose.orientation.w = self.orientation[3]
        self.marker_pub.publish(bebop_marker)

    def publish_landmark(self, event):

        landmark_marker = self.populate_marker()

        landmark_marker.type = Marker.CUBE_LIST
        landmark_marker.id = 1

        for landmark in self.landmarks:
            point = Point()
            point.x = landmark[0]
            point.y = landmark[1]
            point.z = landmark[2]
            landmark_marker.points.append(point)
        
        landmark_marker.color.r = 0
        
        landmark_marker.pose.position.x = 0  # Set the position of the marker
        landmark_marker.pose.position.y = 0
        landmark_marker.pose.position.z = 0
        
        landmark_marker.scale.x = 0.2
        landmark_marker.scale.y = 0.2
        landmark_marker.scale.z = 0.2
        
        self.marker_pub.publish(landmark_marker)

    def add_observation(self, point_list):
        #print("Points:", points)
        _marker = self.populate_marker()
        line_marker = deepcopy(_marker)
      
        line_marker.type = Marker.LINE_STRIP
        line_marker.action=Marker.ADD
        line_marker.id = 30
        
        for point in point_list:
            line_marker.points.append(point)
        
        line_marker.pose.position.x = line_marker.pose.position.y= line_marker.pose.position.z = 0.0
        
        line_marker.scale.x = line_marker.scale.y = line_marker.scale.z = 0.05
        line_marker.color.g = 0
       

        self.marker_pub.publish(line_marker)

 
    def populate_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Set the frame ID as needed
        marker.header.stamp = rospy.Time.now()
        marker.ns = "bebop2"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.position.x = self.position[0]  # Set the position of the marker
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.x = 0.0  # Set the orientation of the marker
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001

        marker.color.r = 0.66
        marker.color.g = 0.66
        marker.color.b = 0.66
        marker.color.a = 1.0

        # Specify the mesh file path (replace with your own mesh file)
        marker.mesh_resource = "package://bebop_description/meshes/bebop_model.stl"
        return marker
    
    def estimated_landmarks(self, z, xEst):
        robot_landmark_coord = map(convert_spherical_to_cartesian, z)
        point_list = []
        for x in robot_landmark_coord:
                
        # self.xEst is not available.
            x0 = np.squeeze(xEst)[:3]
            l0 = x0 + x[:3]
            # print(l0)
            detectedLandmark = Point()
            detectedLandmark.x = l0[0]
            detectedLandmark.y = l0[1]
            detectedLandmark.z = l0[2]

            robotPosition = Point()
            robotPosition.x = x0[0]
            robotPosition.y = x0[1]
            robotPosition.z = x0[2]

            point_list.append(robotPosition)
            point_list.append(detectedLandmark)
            point_list.append(detectedLandmark)
            point_list.append(robotPosition)

        self.add_observation(point_list)
    
    
    def plot_covariance_ellipse(self, xEst, PEst):  # pragma: no cover
        Pxy = PEst[0:2, 0:2]
        eig_val, eig_vec = np.linalg.eig(Pxy)

        if eig_val[0] >= eig_val[1]:
            big_ind = 0
            small_ind = 1
        else:
            big_ind = 1
            small_ind = 0

        t = np.arange(0, 2 * math.pi + 0.1, 0.1)

        # eig_val[big_ind] or eiq_val[small_ind] were occasionally negative
        # numbers extremely close to 0 (~10^-20), catch these cases and set
        # the respective variable to 0
        try:
            a = math.sqrt(eig_val[big_ind])
        except ValueError:
            a = 0

        try:
            b = math.sqrt(eig_val[small_ind])
        except ValueError:
            b = 0

        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        angle = math.atan2(eig_vec[1, big_ind], eig_vec[0, big_ind])
        fx = np.stack([x, y]).T @ rot_mat_2d(angle)

        px = np.array(fx[:, 0] + xEst[0, 0]).flatten()
        py = np.array(fx[:, 1] + xEst[1, 0]).flatten()


        cov_marker = self.populate_marker()

        cov_marker.type = Marker.LINE_STRIP
        cov_marker.id = 135


        for xx, yy in zip(px, py):
            zz = xEst[2, 0]
            point = Point()

            point.x = xx
            point.y = yy
            point.z = zz

            cov_marker.points.append(point)
        
        
        cov_marker.pose.position.x = 0  # Set the position of the marker
        cov_marker.pose.position.y = 0
        cov_marker.pose.position.z = 0
        cov_marker.scale.x = 0.05
        cov_marker.scale.y = 0.05
        cov_marker.scale.z = 0.05
        cov_marker.color.b = 0
        self.marker_pub.publish(cov_marker)