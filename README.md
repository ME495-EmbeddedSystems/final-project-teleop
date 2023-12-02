# Teleoperation Final Project

#### Group Members:
- Rohan Kota
- Luke Batteas
- Zach Alves
- Aditya Nair
- Leo Chen

## Packages:
- [`teleop`](#teleop_tasks-package)
- [`teleop_tasks`](#teleop_tasks-package)
- [`teleop_haptics`](#teleop_haptics-package)
- [`teleop_sensing`](#teleop_sensing-package)
- [`teleop_visualization`](#teleop_visualization-package)
- [`teleop_avatar`](#teleop_avatar-package)
- [`teleop_interfaces`](#teleop_interfaces-package)

## `teleop_tasks` Package

The `teleop_tasks` package defines and launches virtual tasks, which can be completed by the user.

The video below shows an example of a user completing a ring stacking task. The user can feel the mass of the rings due to the Franka robots attached to their hands. The HaptX gloves allow the user to feel the rings in their hands by activating the finger brakes and inflating the tactors when the user picks up a ring.

   https://github.com/ME495-EmbeddedSystems/final-project-teleop/assets/122302059/b09ece30-6af2-4751-981c-6e6976359e5a

## `teleop_haptics` Package

The `teleop_haptics` package can be used to provide haptic feedback to the user. There are 2 main forms of haptic feedback:

- __Kinesthetic Feedback__: Provided by a Franka robot attached to the user's wrist, the kinesthetic feedback allows a user to experience virtual masses when picking up objects.
- __Cutaneous Feedback__: Provided by the HaptX gloves, the cutaneous feedback allows a user to feel objects on their fingertips.

## `teleop_sensing` Package

The `teleop_sensing` package uses computer vision to calculate the locations of objects present in the avatar robot's workspace.

## `teleop_visualization` Package

The `teleop_visualization` package allows you to visualize the avatar's workspace in RViz.

__Real Life:__
![41BA1E21-AB32-4C08-83C6-C6F4C3C7DDBD](https://github.com/ME495-EmbeddedSystems/final-project-teleop/assets/122302059/3c90d238-014b-45b9-bd0b-a8401322f335)

__RViz Visualization:__
![Screenshot from 2023-11-30 15-52-21](https://github.com/ME495-EmbeddedSystems/final-project-teleop/assets/122302059/cb54bf83-4209-43e2-9299-f436f003a32e)

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