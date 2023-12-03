# Teleoperation Final Project

#### Group Members:
- Rohan Kota
- Luke Batteas
- Zach Alves
- Aditya Nair
- Leo Chen

<br>

## Packages:
- [`teleop`](#teleop_tasks-package)
- [`teleop_tasks`](#teleop_tasks-package)
- [`teleop_haptics`](#teleop_haptics-package)
- [`teleop_sensing`](#teleop_sensing-package)
- [`teleop_visualization`](#teleop_visualization-package)
- [`teleop_avatar`](#teleop_avatar-package)
- [`teleop_interfaces`](#teleop_interfaces-package)

<br>

## `teleop_tasks` Package

The `teleop_tasks` package defines and launches virtual tasks, which can be completed by the user.

The video below shows an example of a user completing a ring stacking task. The user can feel the mass of the rings due to the Franka robots attached to their hands. The HaptX gloves allow the user to feel the rings in their hands by activating the finger brakes and inflating the tactors when the user picks up a ring.

   https://github.com/ME495-EmbeddedSystems/final-project-teleop/assets/122302059/b09ece30-6af2-4751-981c-6e6976359e5a

<br>

## `teleop_haptics` Package

The `teleop_haptics` package can be used to provide haptic feedback to the user. There are 2 main forms of haptic feedback:

- __Kinesthetic Feedback__: Provided by a Franka robot attached to the user's wrist, the kinesthetic feedback allows a user to experience virtual masses when picking up objects.
- __Cutaneous Feedback__: Provided by the HaptX gloves, the cutaneous feedback allows a user to feel objects on their fingertips.

<br>

## `teleop_sensing` Package

The `teleop_sensing` package uses computer vision to calculate the locations of objects present in the avatar robot's workspace.

<br>

## `teleop_visualization` Package

The `teleop_visualization` package allows you to visualize the avatar's workspace in RViz.

__Real Life:__
![Avatar Station with Rings](https://github.com/ME495-EmbeddedSystems/final-project-teleop/assets/122302059/c47e4826-eb94-4f62-a949-32af5adda925)

__RViz Visualization:__
![Screenshot from 2023-11-30 15-52-21](https://github.com/ME495-EmbeddedSystems/final-project-teleop/assets/122302059/cb54bf83-4209-43e2-9299-f436f003a32e)

<br>

## `teleop_avatar` Package

The `teleop_avatar` package controls the avatar robot to pick up objects and move them. In order to pick up an object, you can call a service specifying the object's id.

Here is an example of the teleop_avatar package picking up rings and stacking them on a peg:

   https://github.com/ME495-EmbeddedSystems/final-project-teleop/assets/122302059/af606b0f-0a41-4838-8deb-db817e902a9b

<br>

## `teleop_interfaces` Package

This package contains all the custom messages and services used by the teleoperation system.

#### Custom Messages:
- FingerWrenches
- ObjectState

#### Custom Services:
- Grasp
- ExecuteTrajectory
- SetWrench