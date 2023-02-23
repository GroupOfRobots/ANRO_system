# Highly-parametrizable Dobot Magician URDF visualization   

To make maintenance easier, the **magician.urdf.xacro** takes arguments that specify whether the robot has 3 or 4 doF, whether camera is mounted on the robot and type of end effector. 
  
The URDF is automatically generated in a launch file.  
  
## Launch RViZ2 vizualization

ros2 launch dobot_description display.launch.py  
ros2 launch dobot_description display.launch.py --show-arguments

## Example

`ros2 launch dobot_description display.launch.py DOF:=4 tool:=extended_gripper use_camera:=true gui:=true`

## Disclaimer:
The movement in the third axis is limited to a smaller range when using joint_state_publisher_gui than the actual movement of the real robot. This is because in the URDF the rotation constraints are expressed in the joint frame that moves as the manipulator moves. In the Dobot Magician manipulator, the rotation limits of the third axis are expressed in relation to the straight line parallel to the X axis of the manipulator base frame and passing through the third axis of the arm.

## Joint limits: 
* **JT1**: -125° <-> 125°
* **JT2**: -5° <-> 90°
* **JT3**: -15° <-> 70°
* **JT4**: -150° <-> 150°

## Convert Xacro to URDF:
ros2 run xacro xacro <xacro_filename>.urdf.xacro > <new_urdf_filename>.urdf

## Info:
  
http://wiki.ros.org/xacro - **XACRO datasheet**
  
https://answers.ros.org/question/373824/ros2-launch-file-arguments-subsititutions-xacro-and-node-parameters/ - **XACRO arg from Python launch file**

https://answers.ros.org/question/322874/ros2-what-is-different-between-declarelaunchargument-and-launchconfiguration/ - **LogInfo** 
  
https://github.com/ros2/launch/tree/master/launch/launch - **Launch File source repo**