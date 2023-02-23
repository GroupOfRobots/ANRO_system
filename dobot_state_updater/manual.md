# Dobot State Updater

## Running:
ros2 run dobot_state_updater state_publisher

## Topics:
- /dobot_joint_states
- /joint_states
- /dobot_TCP
- /dobot_alarms

## Transforms:
- transformation between _"magician_base_link"_ and _"TCP"_

## Sliding rail:
If you have _Dobot Magician Sliding Rail_ and you want to get real time feedback about the position of the carriage you need to export `MAGICIAN_RAIL_IN_USE` environment variable before starting this node.   
`export MAGICIAN_RAIL_IN_USE=true`  
The current position of the carriage on the sliding rail will be published on the **/dobot_rail_pose** topic at a frequency of 20 Hz.   
After disconnecting the sliding rail, continue your work in a new terminal window or type `unset MAGICIAN_RAIL_IN_USE`. 
