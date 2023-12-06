# Teleoperation Final Project

#### Group Members:
- Rohan Kota
- Luke Batteas
- Zach Alves
- Aditya Nair
- Leo Chen

<br>

## Overview:

The goal of this project was to design a teleoperation system which leverages object models in order to make the user's experience more intuitive. The idea is that a human operator can stack virtual rings in a simulated environment, all while getting instantaneous haptic feedback thanks to the local object models and low-latency simulation environment. The user's impact on the world is then replicated by an avatar robot, i.e. the robot does the same thing with the rings that the human user did. While we did not end up bridging the operator station and avatar station, we developed several key packages which will contribute to this end goal. These packages are outlined below, and more detailed information about how to run each package, etc. can be found in the READMEs of each individual package.

<br>

## Packages:

This projects consists of the following packages:
- [`teleop`](#teleop_tasks-package)
- [`teleop_tasks`](#teleop_tasks-package)
- [`teleop_haptics`](#teleop_haptics-package)
- [`teleop_sensing`](#teleop_sensing-package)
- [`teleop_visualization`](#teleop_visualization-package)
- [`teleop_avatar`](#teleop_avatar-package)
- [`teleop_interfaces`](#teleop_interfaces-package)

__**Instructions on how to run/launch nodes in each package can be found within the READMEs of the individual packages.**__

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

A demo of the package's ability to detect the positions of the rings is shown below:

   https://github.com/ME495-EmbeddedSystems/final-project-teleop/assets/7969697/c79e4f70-6d18-47e0-8438-9ad4425930bc

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

   https://github.com/ME495-EmbeddedSystems/final-project-teleop/assets/122302059/d2059ccf-5fb9-461d-afd6-dd741a69f29b

<br>

## `teleop_interfaces` Package

The `teleop_interfaces` package contains all the custom messages and services used by the teleoperation system.

#### Custom Messages:
- FingerWrenches
- ObjectState

#### Custom Services:
- Grasp
- ExecuteTrajectory
- SetWrench
