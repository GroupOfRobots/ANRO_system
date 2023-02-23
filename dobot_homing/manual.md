# Dobot homing procedure:

## Running server (params are NOT SET):
ros2 run dobot_homing homing_server   

## Launching server (params are SET using config .yaml file):
ros2 launch dobot_homing dobot_homing.launch.py   

## Running client (alternative ways):
ros2 service call /dobot_homing_service dobot_msgs/srv/ExecuteHomingProcedure  
rqt -> Service Caller Plugin  


## Perform parameter operations via parameter services from external processes (2 of 5 services below): 
ros2 service call /dobot_homing_srv/get_parameters rcl_interfaces/srv/GetParameters "{names: ['homing_position']}"  
ros2 service call /dobot_homing_srv/describe_parameters rcl_interfaces/srv/DescribeParameters "{names: ['homing_position']}"  

## Parameters via the command-line:
ros2 param list  
ros2 param get /dobot_homing_srv homing_position  
ros2 param set /dobot_homing_srv homing_position [150.0,0.0,100.0,0.0]


-------------------------------------

## ExecuteProcess in LaunchDescription
Parameters are loaded by calling a `ros2 param load` command using ExecuteProcess() function so that parameters_callback() will be called that will send the robot a command to change the homing parameters.

## Service structure reference:
http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html

## Slash in YAML file: 
https://github.com/ros2/ros2cli/pull/600