# dobot_bringup package

## Sources:
- https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-Main.html  
- https://github.com/GroupOfRobots/Minirys_ws/tree/master/minirys_ros2/launch  
- https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/bringup_launch.py  
- https://github.com/ros2/launch/tree/humble/launch/launch

## Running: 
1. Declare tool type:
`export MAGICIAN_TOOL=<tool_type>`  
Possible values: _none, pen, suction_cup, gripper, extended_gripper_  
2. If you want to use sliding rail, type `export MAGICIAN_RAIL_IN_USE=true`  
3. Launch control system: 
`ros2 launch dobot_bringup dobot_magician_control_system.launch.py`
