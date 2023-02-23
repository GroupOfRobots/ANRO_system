# Gripper and suction cup control 

## Show msg/srv structure
ros2 interface show dobot_msgs/srv/GripperControl  
ros2 interface show dobot_msgs/msg/GripperStatus  
ros2 interface show dobot_msgs/srv/SuctionCupControl 

  
## Call service from console  
ros2 service call /dobot_gripper_service dobot_msgs/srv/GripperControl "{gripper_state: 'open', keep_compressor_running: true}"
  
ros2 service call /dobot_suction_cup_service dobot_msgs/srv/SuctionCupControl "{enable_suction: true}"


## Call service from RQT  
rqt -> Service Caller