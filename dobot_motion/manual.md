# Point to point movement

## Launch server node:  
ros2 launch dobot_motion dobot_PTP.launch.py 

## Send goal to action server:
ros2 action send_goal /PTP_action  dobot_msgs/action/PointToPoint "{motion_type: 4, target_pose: [0.0, 24.0, 10.0, 0.0]}" --feedback 
  
ros2 action send_goal /PTP_action  dobot_msgs/action/PointToPoint "{motion_type: 1, target_pose: [200.0, 0.0, 100.0, 0.0]}" --feedback 

## Optional fields in action goal definition:  
You can define:
* velocity ratio (default 1.0)
* acceleration ratio (default 1.0)

ros2 action send_goal /PTP_action  dobot_msgs/action/PointToPoint "{motion_type: 1, target_pose: [200.0, 0.0, 100.0, 0.0], velocity_ratio: 0.5, acceleration_ratio: 0.3}" --feedback


## Cancel goal:
ros2 service call /PTP_action/_action/cancel_goal action_msgs/srv/CancelGoal

## Parameters (command line tools):
ros2 param list    
ros2 param get /dobot_PTP_server JT3_vel   
ros2 param set /dobot_PTP_server JT3_vel 80    
ros2 param dump /dobot_PTP_server  
ros2 param describe /dobot_PTP_server TCP_vel  

## Influence of parameters on the movement of Dobot Magician arm:
* JT1_vel, JT2_vel, JT3_vel, JT4_vel, JT1_acc, JT2_acc, JT3_acc, JT4_acc parameters have an influence on the speed of motion interpolated in the **joint space** (motion type includes **MOVJ**).
* TCP_vel, end_tool_rot_vel, TCP_acc, end_tool_rot_acc parameters have an influence on the speed of motion interpolated in the **cartesian space** (motion type includes **MOVL**).



## Action (command line tools):
ros2 action info /PTP_action      
ros2 action info /PTP_action -t [display type of msg]  
ros2 action list  
ros2 action send_goal /PTP_action  --help
ros2 interface show dobot_msgs/action/PointToPoint
ros2 topic list --include-hidden-topics 
ros2 service --include-hidden-services list


## Suported and tested modes: 
* MOTION_TYPE_MOVJ_XYZ (1)
* MOTION_TYPE_MOVL_XYZ (2)
* MOTION_TYPE_MOVJ_ANGLE (4)
* MOTION_TYPE_MOVL_ANGLE (5)

## Sources:
* Command Line Interface: https://osrf.github.io/ros2multirobotbook/ros2_cli.html
* Server: https://github.com/ros2/examples/blob/rolling/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server.py
* Client: https://github.com/ros2/examples/blob/rolling/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_cancel.py
* https://design.ros2.org/articles/actions.html 
* https://docs.ros2.org/latest/api/rclpy/api/actions.html 
* Callback groups, executors (avoiding _deadlocks_): https://discourse.ros.org/t/how-to-use-callback-groups-in-ros2/25255?fbclid=IwAR0_dAdqU_cpXOmGR7YbrzeiiOtaMk1-vU8ALuKc_EuAcMqeufBlpts4_JA, https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html?fbclid=IwAR2dv2sidqJ-KxxTVXKfKKw2styPkK7IolYU55liMXDoERDrzeO2lUsiUuk 


## Topics:
* /PTP_action/_action/feedback
* /PTP_action/_action/status


## Services:  
* /PTP_action/_action/cancel_goal
* /PTP_action/_action/send_goal

## GoalStatus enum meaning:
https://docs.ros2.org/galactic/api/action_msgs/msg/GoalStatus.html
