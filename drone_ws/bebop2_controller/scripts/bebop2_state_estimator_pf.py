#!/usr/bin/env python  
import rospy
import numpy as np 
from RosInterface import Bebop2StateEstimator

NP = 1000  # Number of Particle
DT = 0.03  # time tick [s]
STATE_DIM = 8 # state variable dimension


# TAG_ID positions [x, y, z]
tag_id = np.array([ [0.81, 7.0, 1.2],
                    [1.96, 7.0, 1.2],
                    [3.11, 7.0, 1.2],
                    [4.26, 7.0, 1.2]])
tag_id = np.expand_dims(tag_id, axis=2)



if __name__ == '__main__':
    rospy.init_node('bebop2_state_estimator')
    bebob2 = Bebop2StateEstimator(tag_id, STATE_DIM, NP, DT)
    rospy.spin()

    



    


  