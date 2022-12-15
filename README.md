# Bebop2 Controller

## Quick Run

Connect your joystick to the computer.
Joystick buttons perform following operations:
```angular2html
    IDLE = 0, // setpoint can be freely moved with joystick. Sphere color blue
    TAKEOFF = 1, // drone will takeoff from the ground
    LAND = 2, //drone will land
    ENGAGE = 3, // set current location as a set point (for hover). Sphere color yellow
    CONTROL = 4 // start PID controller for the current (ENGAGE) setpoint. Sphere color cyan
```

Once control mode is enabled, setpoint can also be controlled using joystick axes.
Note that, only position controller is implemented. 
Orientation controller needs yaw angle which could be directly obtained from bebop odom.

## Parameter Tuning

Refer to ``config/param.yaml`` file to read about tuning parameters.
Apriltags are used for landmarks and then
the drone position is estimated from those landmarks.
A complementary filter is implemented to compute state by combining multiple landmarks.
