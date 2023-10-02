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
  csv_path: '/home/redwan/catkin_ws/src/bebop2_controller/test/wps.csv'
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
  csv_path: '/home/redwan/catkin_ws/src/bebop2_controller/test/wps.csv'
  method: 0
"

rostopic pub --once /waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'goal2'
goal:
  csv_path: '/home/redwan/catkin_ws/src/bebop2_controller/test/wps.csv'
  method: 2
"
