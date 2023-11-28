#!/bin/bash
send_trajectory()
{
rostopic pub --once /bebop/waypoint_action/goal bebop2_controller/WaypointsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  id: 'sqr_exp3'
goal:
  csv_path: '/home/roboticslab/catkin_ws/src/bebop2_controller/test/Square.csv'
  method: 0
"
}

#roslaunch bebop2_controller nav_controller_debug.launch
send_trajectory
# case $1 in
#     "move")
#         echo "You chose a move_to"
#         move_to
#         ;;

      
#     "send")
#         echo "You chose a trajectory"
#         send_trajectory
#         ;;

#     *)
#         echo "Sorry, I don't know that option."
#         ;;
# esac