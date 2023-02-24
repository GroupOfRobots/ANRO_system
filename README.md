# Konfiguracja P109
```
source /opt/ros/humble/setup.bash
mkdir -p ~/magician_ros2_control_system_ws/src
git clone <link-to-github-repository> ~/magician_ros2_control_system_ws/src
cd magician_ros2_control_system_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build
```
(delete all dirs in workspace except /install dir)



* `echo "export ROS_DOMAIN_ID=<id>" >> ~/.bashrc` (number between 0 and 101, inclusive)
* `echo "source /home/student/magician_ros2_control_system_ws/install/setup.bash" >> ~/.bashrc`
* `pip3 install -r requirements.txt`
* `sudo apt install ros-humble-diagnostic-aggregator ros-humble-rqt-robot-monitor python3-pykdl` 
* `rqt --force-discover`
