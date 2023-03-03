# Konfiguracja P109 (instalacja w /opt)
```
cd /opt
su administrator
sudo mkdir dobot_anro_system
download .zip -> Extract Here -> Delete .zip
exit
cd
su administrator
sudo mv Downloads/ANRO_system-main/ /opt/dobot_anro_system/
cd
cd /opt
su administrator
cd dobot_anro_system/
sudo mv ANRO_system-main/ src
source /opt/ros/humble/setup.bash
sudo rosdep init (if necessary)
rosdep update (if necessary)
rosdep install -i --from-path src --rosdistro humble -y
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-diagnostic-aggregator ros-humble-rqt-robot-monitor ros-humble-tf-transformations python3-pykdl python3-pip
pip3 install -r src/requirements.txt
exit
pip3 install -r src/requirements.txt
su administrator
cd ..
sudo rm -r dobot_anro_system/
exit
cd
source /opt/ros/humble/setup.bash
cd dobot_anro_system/
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
colcon build
rm -r build log src 
rm -r src/
delete python scripts from /install (also remove them from Trash)
su administrator 
cd
with File Manager copy
sudo adduser student dialout (restart system to apply changes)
rqt --force-discover

source /opt/dobot_anro_system/install/setup.bash


echo "source /home/student/dobot_anro_system/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=<id>" >> ~/.bashrc (number between 0 and 101, inclusive)
```

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

