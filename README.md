@mainpage Parrot Bebop2 Drone with Joystick Controller README File


## Introduction

The Parrot Bebop 2 is a small, lightweight manufactured by Parrot, a French company specializing in consumer electronics. It is compact and lightweight, making it easy to transport and use on the go which makes it a popular choice 
It's camera has a most impressive electronic image stabilization and with a top speed of 40 mph, this drone can really move. The Bebop 2 has a battery life of up to 25 minutes. <br>

This README file explains how to control the drone using a joystick and ROS drivers, add visual servoing capabilities, and scan for April Tags to find the drone's relative position.

-----------------------------------------

<img src="/home/simant/Undergrad_Research/bebop2_controller_docs/bebop2_controller/Images/img-bebop2.jpg" alt="Parrot Bebop2 Drone" align="right" width="400" height="400">



## Quick Run

**Hardware Setup**: 
Connect your joystick to the computer. <br>


Before starting the program, the drone should be turned on and the computer connected to the drone WiFi.

> **Warning**
> It is strictly recommended to use protective equipments while testing the drone. <br>
If the drone needs to be shut down completely then we should flip the drone.


*Joystick buttons perform following operations*:

```angular2html
    IDLE = 0, // Setpoint can be freely moved with joystick. Sphere color blue

    TAKEOFF = 1, // Drone will takeoff from the ground
    
    LAND = 2, // Drone will land

    ENGAGE = 3, // Set current location as a set point (for hover). Sphere color yellow

    CONTROL = 4 // Start PID controller for the current (ENGAGE) setpoint. Sphere color cyan
```

Once control mode is enabled, setpoint can also be controlled using joystick axes.
Note that, only position controller is implemented from the analog sticks. 
Orientation controller needs yaw angle which could be directly obtained from bebop odom.

------------------------------------------


## Software Setup:

-- A computer with Ubuntu 16.04, 18.04 or above and ROS Kinetic, Melodic or above installed.

-- **Visp_ros** package is needed to do visual servoing with Parrot Bebop 2 drone. Visual servoing, also known as vision-based robot control, is a technique which uses feedback information extracted from a vision sensor (visual feedback) to control the motion of a robot.

-- **Installation of bebop_autonomy** from fork manually is required to send relative move commands to the drone. Check out [install bebop_autonomy from fork and build visp](http://wiki.ros.org/visp_ros/Tutorials/How%20to%20do%20visual%20servoing%20with%20Parrot%20Bebop%202%20drone%20and%20visp_ros). <br>
Bebop_autonomy is a ROS driver for Parrot Bebop 2.0 drones (quadrocopters), based on Parrot’s official ARDroneSDK3. You can check out [this source code](https://github.com/AutonomyLab/bebop_autonomy) for the driver is for more understanding.

---------------------------------------------


<img align="right" width="280" height="200" alt="Joystick Controller" src="/home/simant/Undergrad_Research/bebop2_controller_docs/bebop2_controller/Images/f710-gallery-1.png">

## Operating the drone


The Parrot Bebop 2 Drone can be controlled from a PID Joystick Controller. The drone can be manuvered with the joytsick analog sticks. 

The functions of the Four Action Buttons of the joystick is as follows:
 
// To be written down. 

1.  A   ---> TAKE OFF (Green button)
2.  B   ---> LAND (Red Button)
3.  X   ---> IDLE (Blue Button)
4.  Y   ---> ENGAGE (Yellow Button)
5.  L   ---> CONTROL (Top Left Button) 

---------------------------------------------

<img align="right" width="150" height="150" alt="AprilTag 36h11" src="/home/simant/Undergrad_Research/bebop2_controller_docs/bebop2_controller/Images/tag_36h11.png">

## April Tags


AprilTag is a visual fiducial system, useful for a wide variety of tasks including augmented reality, robotics, and camera calibration. 
Targets can be created from an ordinary printer, and the AprilTag detection software computes the precise 3D position, orientation, and identity of the tags relative to the camera. <br><br>

Using the built-in camera of the Parrot Bebop 2 Drone, it scans the tag and determines its relative postion from the tag. <br>


An April Tag from 36h11 family that will serve as target for the visual servoing (see [this page](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-franka-pbvs.html#franka_prereq_target) to print one).



---------------------------------------------

## Visualisation using Graphical Interface (RVIZ)

The Parrot Bebop 2 drone can be monitored and visualised using a graphical interface like rviz. <br>

**For RVIZ installtion** :-
```
    sudo apt-get install ros-[ros-version]-rviz 
```

For example:-
```
    sudo apt-get install ros-noetic-rviz
```

To run rviz 
```
rosrun rviz rviz

```

Check out http://wiki.ros.org/rviz/UserGuide for more information.

-----------------------------------------------

## Parameter Tuning

Refer to ``config/param.yaml`` file to read about tuning parameters.
Apriltags are used for landmarks and then
the drone position is estimated from those landmarks.
A complementary filter is implemented to compute state by combining multiple landmarks.
