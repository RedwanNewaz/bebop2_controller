#!/usr/bin/env bash

##"ros2", "action", "send_goal", "<NS>/waypoints", "action_waypoints_interfaces/action/Waypoints",  "\{csv_path: <INP> \}"
#rostopic pub /waypoint_action/goal bebop2_controller/WaypointsActionGoal  "{goal:{csv_path:'/home/redwan/catkin_ws/src/bebop2_controller/test/wps.csv'}}"
#
rostopic pub /waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: 'goal1'
goal:
  csv_path: '/home/redwan/catkin_ws/src/bebop2_controller/test/wps.csv'
  tracking_sequence: [1.0, 2.0, 3.0]
  time_sequence: [0.0, 1.0, 2.0]
"
