# `teleop_haptics` package

The `teleop_haptics` package enables two forms of haptic feedback:
- __Cutaneous Feedback:__ Provided by the HaptX Gloves, cutaneous feedback allows a user to feel contact on their fingertips.
- __Kinesthetic Feedback:__ Provided by the Franka robot arms, kinesthetic feedback helps to simulate the masses/intertias of objects.

<br>

### Launch The Haptic Feedback

To launch both the cutaneous feedback and the kinesthetic feedback, you can use the following command:

```bash
ros2 launch teleop_haptics haptics.launch.xml
```

This launch file takes the following arguments:
- `side`: Determines whether to launch haptic feedback for the right side, left side, or both sides
    - __Default__: right
    - __Options__: right, left, both
- `track`: Determines whether hand tracking from the frankas should be published __(default: true)__

For example, to launch feedback for the both sides of the body, you can use the following command:

```bash
ros2 launch teleop_haptics haptics.launch.xml side:=both
```

To launch just force feedback, the launch file `force.launch.xml` exists.
This launch file takes the following arguments:
- `side`: Determines whether to launch haptic feedback for the right side, left side, or both sides
    - __Default__: right
    - __Options__: right, left, both
- `track`: Determines whether hand tracking from the frankas should be published __(default: true)__

This can be used with the following command:
```bash
ros2 launch teleop_haptics force.launch.xml 
```

<br>

### Cutaneous Feedback Node

To run just the cutaneous feedback node, you can use the following command:

```bash
ros2 run teleop_haptics haptx_feedback
```

This node takes in one parameter:
- `side`: Determines whether to launch haptic feedback for the right side, left side, or both sides
    - __Default__: right
    - __Options__: right, left, both

For example, to run just the left HaptX Glove, you can use the following command:

```bash
ros2 run teleop_haptics haptx_feedback --ros-args -p side:=left
```

<br>

### Kinesthetic Feedback Node
The kinesthetic feedback node provides force feedback to the hands. 

To run the just kinesthetic feedback node, you can use the following command:

```bash
ros2 run teleop_haptics force_feedback
```

### Hand Positioner Node
The hand posiitoner node provides an alternative to vive hand tracking.
It takes in several parameters which control the output of the hand position.
 - `z_offset` : The z offset of the hand from the end effcetor. The default is 0.0
 - `yaw_offset` : The yaw offset of the hand from the end effector. The default is -1.57 radians.
 - `x_world_offset` : The z offset of the world frame from the franka base link. The default is 0.0
 - `y_world_offset` : The z offset of the world frame from the franka base link. The default is 0.0
 - `z_world_offset` : The z offset of the world frame from the franka base link. The default is 0.0

 To run the just hand positioner node, you can use the following command:

```bash
ros2 run teleop_haptics hand_position
```