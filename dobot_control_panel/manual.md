# Dobot Contol Panel - RQT Plugin

## plugin.xml file explained
* http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin


## Separate node for params:
The name of the ROS rqt plugin node consists of the prefix "rqt_gui_py_node_" and the process id. In the YAML file containing the configuration parameters, you need to specify the name of the node, which in this particular case is not possible to predict.  
https://github.com/ros-visualization/rqt/blob/ros2/rqt_gui_py/src/rqt_gui_py/plugin.py

## Build
colcon build --symlink-install --packages-select dobot_control_panel
. install/setup.bash

## Running:
rqt --standalone dobot_control_panel


## Troubleshooting 
rqt --force-discover  
(https://answers.ros.org/question/338282/ros2-what-is-the-rqt-force-discover-option-meaning/)

## Tutorial
https://github.com/ChoKasem/rqt_tut?fbclid=IwAR2Cpqk5fwB0ueyMuVVJwvoa_7Bytwg9PvtUpo0otAt5CltGs4QHZB9zDgY 

## Examples:
* Velma Control Panel -> https://github.com/RCPRG-ros-pkg/velma_system/blob/melodic-devel/rqt_velma/src/rqt_velma/velma_control_panel_widget.py
* OpenMANIPULATOR-X Controller  -> https://github.com/ROBOTIS-GIT/open_manipulator/tree/master/open_manipulator_control_gui 
* rqt_robot_dashboard -> https://github.com/ros-visualization/rqt_robot_dashboard
* rqt_robot_steering -> https://github.com/ros-visualization/rqt_robot_steering
* rqt_pr2_dashboard -> https://github.com/PR2/rqt_pr2_dashboard
* ROS rqt plugin for turtlesim -> https://fjp.at/ros/rqt-turtle/

## How to Create Custom RQT Plugin for Dummies:
* https://github.com/ChoKasem/rqt_tut

## Steps to create ROS RQT plugin: 
* https://github.com/BruceChanJianLe/ros-rqt-plugin

## Create your new rqt plugin:
* http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin

## Working ROS2 plugin:
* https://github.com/flobotics/ros2_galactic_rqt_rt_preempt

## Functionalities:
* E-STOP -> https://github.com/nrjl/rqt_estop
* Connection status -> https://wiki.ros.org/pr2_dashboard
