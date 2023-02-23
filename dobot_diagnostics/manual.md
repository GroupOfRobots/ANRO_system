# Diagnostics package

## Prerequisites:
sudo apt install ros-humble-diagnostic-aggregator  
sudo apt install ros-humble-rqt-robot-monitor

## Helpful information: 
After installing rqt-robot-monitor, run **rqt --force-discover** command to make the plugin visible in the rqt GUI.

## Source:
https://github.com/ros/diagnostics/tree/galactic/diagnostic_aggregator 

## Bibliography:
* http://wiki.ros.org/diagnostics 
* https://nlamprian.me/blog/software/ros/2018/03/21/ros-diagnostics/


## Running: 
ros2 launch dobot_diagnostics alarms_analyzer.launch.py   
rqt -> Plugins -> Robot Tools -> Diagnostics Viewer  


## Debugging/Testing:  
ros2 topic pub dobot_alarms dobot_msgs/msg/DobotAlarmCodes "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: }, alarms_list: [3,32,46]}"
