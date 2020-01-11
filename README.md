# espeleo_return
----------------------
This repository contains the codes for the local autonomous navigation for radio connection reestablishment.




## Scripts included on this package:
- lista_position_return_distance.py: This script is responsible to record the path traveled by the robot in an array/buffer and also for sending this array to the script vec_field_control.py when the radio connection is lost.
- vec_field_control.py: This script implements, via Vector Field and Feedback Linearization, the algorithm that makes the robot travel the path given by script above.



## How to interact

When the radio connection is lost, this code implements the algorithm that makes the robot return autonomously to a certain point known to have a radio connection. In order to the code runs properly, it is needed to subscribe to the positions of the robot. The code now is implemented using position data from the topic /tf.

**Topics:**
Published Topics
- `/cmd_vel`  (message type:`geometry_msgs.msg/Twist`)
- `/return/traj_points` (message type:`geometry_msgs.msg/Polygon`)
- `/visualization_marker_array` (message type:`visualization_msgs.msg/MarkerArray`)
- `/visualization_marker_ref` (message type:`visualization_msgs.msg/Marker`)
- `/flag/distance_target` (message type:`std_msgs.msg/Bool`)

**Launch file**
- `return.launch`


**Input parameters:**
These parameters are found in the initial section of the script lista_position_return_distance.py
- delta: Minimum variance in the position of the robot to record the new position.
- tolerance: Tolerance between the final point of stop set by the array and the actual position of the robot.
- size: Distance stored in the array in meters.

