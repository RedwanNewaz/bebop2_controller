#!/usr/bin/env bash

rostopic pub --once /waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'goal1'
goal:
  csv_path: '/home/airlab/catkin_ws/src/bebop2_controller/test/wps.csv'
  method: 1
"


rostopic pub --once /waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'goal4'
goal:
  csv_path: '/home/airlab/catkin_ws/src/bebop2_controller/test/wps.csv'
  method: 0
"
## min snap
rostopic pub --once /waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'goal2'
goal:
  csv_path: '/home/airlab/catkin_ws/src/bebop2_controller/test/eight.csv'
  method: 2
"

## min jerk
rostopic pub --once /waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'goalMinJerk'
goal:
  csv_path: '/home/airlab/catkin_ws/src/bebop2_controller/test/eight.csv'
  method: 1
"



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
    x: 2.5
    y: 3.5
    z: 1.2
  method: 0
"
#3.500000, 2.500000, 1.200000

rostopic pub --once /move_p2p/goal bebop2_controller/SetpointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'goal03'
goal:
  setpoint:
    x: 3.5
    y: 2.5
    z: 1.2
  method: 0
"