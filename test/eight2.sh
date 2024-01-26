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
  id: 'wps1'
goal:
  setpoint:
    x: 2.5
    y: 4.0
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
  id: 'constAcc04'
goal:
  csv_path: '/home/roboticslab/CppDev/TrajViewer/vicon_eight_traj.csv'
  method: 0
"
}


send_trajectory2()
{
rostopic pub --once /bebop/waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'minSnap2'
goal:
  csv_path: '/home/roboticslab/CppDev/TrajViewer/vicon_eight_traj.csv'
  method: 2
"
}


send_trajectory2()
{
rostopic pub --once /waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'minSnap2'
goal:
  csv_path: '/home/roboticslab/CppDev/TrajViewer/vicon_eight_traj.csv'
  method: 2
"
}


send_unittest()
{
rostopic pub --once /waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
 seq: 0
 stamp:
   secs: 0
   nsecs: 0
 frame_id: ''
goal_id:
 id: 'goalMinJerk4'
goal:
 csv_path: '/home/redwan/catkin_ws/src/bebop2_controller/test/logger_2023-10-03_16:43:44.csv'
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