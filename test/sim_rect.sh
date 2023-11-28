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
rostopic pub --once /waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'sqr_sim_1'
goal:
  csv_path: '/home/roboticslab/catkin_ws/src/bebop2_controller/test/Square.csv'
  method: 0
"
}
# move_to
send_trajectory
