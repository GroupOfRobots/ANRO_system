# Checklist :white_check_mark:

## Connection test :star:
```
ros2 run dobot_bringup connection_test 
```

## dobot_bringup :star:
```
ros2 launch dobot_bringup dobot_magician_control_system.launch.py
```

## dobot_control_panel :star:
```
rqt -s dobot_control_panel
```

## dobot_diagnostics  :star:
```
rqt -s rqt_robot_monitor
```

## dobot_end_effector :star:
Sprawdzić wszystkie kombinacje: 
```
ros2 service call /dobot_gripper_service dobot_msgs/srv/GripperControl "{gripper_state: 'open', keep_compressor_running: true}"
```
```
ros2 service call /dobot_suction_cup_service dobot_msgs/srv/SuctionCupControl "{enable_suction: true}"
```
## dobot_homing :star:
```
ros2 service call /dobot_homing_service dobot_msgs/srv/ExecuteHomingProcedure
```
```
ros2 param get /dobot_homing_srv homing_position  
ros2 param set /dobot_homing_srv homing_position [150.0,0.0,100.0,0.0]
```
## dobot_kinematics :star:
Sprawdzić:
* (False, 'Joint limits violated')
* (True, 'Trajectory is safe and feasible.')
* (False, 'Inv Kin solving error!')

## dobot_motion  :star:
- wszystkie tryby (4) 
- modyfikacja ratio 
```
ros2 action send_goal /PTP_action  dobot_msgs/action/PointToPoint "{motion_type: 1, target_pose: [200.0, 0.0, 100.0, 0.0], velocity_ratio: 0.5, acceleration_ratio: 0.3}" --feedback
```
```
ros2 service call /PTP_action/_action/cancel_goal action_msgs/srv/CancelGoal
```
```
ros2 param list    
ros2 param get /dobot_PTP_server JT3_vel   
ros2 param set /dobot_PTP_server JT3_vel 80    
ros2 param describe /dobot_PTP_server TCP_vel 
```
## dobot_msgs :star:
```
ros2 interface show <msg_type>
```
## dobot_state_updater :star:
```
ros2 topic echo <topic_name>
```

