# `teleop_haptics` package

The `teleop_haptics` package enables two forms of haptic feedback:
- __Cutaneous Feedback:__ Provided by the HaptX Gloves, cutaneous feedback allows a user to feel contact on their fingertips.
- __Kinesthetic Feedback:__ Provided by the Franka robot arms, kinesthetic feedback helps to simulate the masses/intertias of objects.

<br>

## Launch The Haptic Feedback

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

<br>

## Cutaneous Feedback Node

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

## Kinesthetic Feedback Node

To run the just kinesthetic feedback node, you can use the following command:

```bash
ros2 run teleop_haptics force_feedback
```