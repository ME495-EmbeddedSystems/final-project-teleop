To run the blocks simulation run "ros2 launch teleop_tasks rings.launch.xml"
To run the blocks simulation run "ros2 launch teleop_tasks blocks.launch.xml"

Frames of the desired objects
Block frames: childID -> block
Ring frames: childID -> ring_red, ring_orange, ring_yellow

Force and torque info in this topic: /force_torque
Published as a stamped wrench type
Example published wrench:
header:
  stamp:
    sec: 2
    nanosec: 60000000
  frame_id: left_hand/index_3_index_tip/force_torque_sensor
wrench:
  force:
    x: -2.6198082581022587e-07
    y: 8.054420889837468e-06
    z: 0.9799984506689391
  torque:
    x: 1.662303241172108e-11
    y: -3.2122527912891555e-07
    z: -3.328256418488998e-11