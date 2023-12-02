# Teleoperation Final Project

#### Group Members:
- Rohan Kota
- Luke Batteas
- Zach Alves
- Aditya Nair
- Leo Chen

## Packages:
- #teleop_tasks-package
- `teleop_tasks`
- `teleop_haptics`
- `teleop_sensing`
- `teleop_visualization`
- `teleop_avatar`
- `teleop_interfaces`

## `teleop_tasks` Package

The `teleop_tasks` package defines and launches virtual tasks, which can be completed by the user.

The video below shows an example of a user completing a ring stacking task. The user can feel the mass of the rings due to the Franka robots attached to their hands. The HaptX gloves allow the user to feel the rings in their hands by activating the finger brakes and inflating the tactors when the user picks up a ring.

   https://github.com/ME495-EmbeddedSystems/final-project-teleop/assets/122302059/da3c07da-fc37-454f-bdcc-ac4ab7e976cd

## `teleop_haptics` Package

The `teleop_haptics` package can be used to provide haptic feedback to the user. There are 2 main forms of haptic feedback:

- __Kinesthetic Feedback__: Provided by a Franka robot attached to the user's wrist, the kinesthetic feedback allows a user to experience virtual masses when picking up objects.
- __Cutaneous Feedback__: Provided by the HaptX gloves, the cutaneous feedback allows a user to feel objects on their fingertips.

## `teleop_sensing` Package

The `teleop_sensing` package uses computer vision to calculate the locations of objects present in the avatar robot's workspace.

## `teleop_visualization` Package

The `teleop_visualization` package allows you to visualize the avatar's workspace in RViz.

## `teleop_avatar` Package

The `teleop_avatar` package controls the avatar robot to pick up objects and move them. In order to pick up an object, you can call a service specifying the object's id.

## `teleop_interfaces` Package

This package contains all the custom messages and services used by the teleoperation system.

### Custom Messages
- FingerWrenches
- ObjectState

### Custom Services
- Grasp
- ExecuteTrajectory
- SetWrench