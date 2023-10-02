#!/bin/bash
move_to()
{
rostopic pub --once /move_p2p/goal bebop2_controller/SetpointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'wps'
goal:
  setpoint:
    x: 3.5
    y: 3.5
    z: 1.2
  method: 0
"
}

send_trajectory()
{
rostopic pub --once /waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'goalMinJerk'
goal:
  csv_path: '/home/airlab/catkin_ws/src/bebop2_controller/test/eight2.csv'
  method: 1
"
}

#roslaunch bebop2_controller nav_controller_debug.launch

case $1 in
    "move")
        echo "You chose a move_to"
        move_to
        ;;
    "send")
        echo "You chose a trajectory"
        send_trajectory
        ;;

    *)
        echo "Sorry, I don't know that option."
        ;;
esac