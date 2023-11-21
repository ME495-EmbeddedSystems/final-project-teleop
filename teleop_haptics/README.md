# teleop_haptics package

The `teleop_haptics` package enables two forms of haptic feedback:
- __Cutaneous Feedback:__ Provided by the HaptX Gloves, cutaneous feedback allows a user to feel contact on their fingertips.
- __Kinesthetic Feedback:__ Provided by the Franka robot arms, kinesthetic feedback helps to simulate the masses/intertias of objects.

<br>

## Launch Everything

To launch both the cutaneous feedback and the kinesthetic feedback, you can use the following command:

```bash
ros2 launch teleop_haptics haptics.launch.xml
```

This launch file takes the following arguments:
- `use_right`: Determines whether or not the right side is being used __(default: true)__
- `use_left`: Determines whether or not the left side is being used __(default: true)__

For example, to launch feedback for the right side of the body only, you can use the following command:

```bash
ros2 launch teleop_haptics haptics.launch.xml use_left:=false
```

<br>

## Cutaneous Feedback Node

To run the cutaneous feedback node, you can use the following command:

```bash
ros2 run teleop_haptics haptx_feedback
```

This node takes in two parameters:
- `use_right`: Determines whether or not the right HaptX glove is being used __(default: true)__
- `use_left`: Determines whether or not the left HaptX glove is being used __(default: true)__

For example, to run just the right HaptX Glove, you can use the following command:

```bash
ros2 run teleop_haptics haptx_feedback --ros-args -p use_left:=false
```

<br>

## Kinesthetic Feedback Node

To run the kinesthetic feedback node, you can use the following command:

```bash
ros2 run teleop_haptics force_feedback
```