#!/bin/bash
move_to()
{
rostopic pub --once /bebop/move_p2p/goal bebop2_controller/SetpointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'wps'
goal:
  setpoint:
    x: 2.5
    y: 3.5
    z: 1.0
  method: 0
"
}

move_to2()
{
rostopic pub --once /bebop/move_p2p/goal bebop2_controller/SetpointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'wps1'
goal:
  setpoint:
    x: 3.2
    y: 2.4
    z: 1.0
  method: 0
"
}


move_to3()
{
rostopic pub --once /bebop/move_p2p/goal bebop2_controller/SetpointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'wps2'
goal:
  setpoint:
    x: 1.8
    y: 3.2
    z: 1.0
  method: 0
"
}

send_trajectory()
{
rostopic pub --once /bebop/waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'goalMinJerk'
goal:
  csv_path: '/home/roboticslab/catkin_ws/src/bebop2_controller/test/logger_2023-10-16_15:21:23.csv'
  method: 1
"
}

#roslaunch bebop2_controller nav_controller_debug.launch

case $1 in
    "move")
        echo "You chose a move_to"
        move_to
        ;;

      "move2")
              echo "You chose a move_to"
              move_to2
              ;;

      "move3")
              echo "You chose a move_to"
              move_to3
              ;;

    "send")
        echo "You chose a trajectory"
        send_trajectory
        ;;

    *)
        echo "Sorry, I don't know that option."
        ;;
esac