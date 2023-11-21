# teleop_interfaces package

## Messages

The `teleop_interfaces` package features the following messages:
- `ObjectState`: Used to publish the poses of moving objects

### teleop_interfaces/ObjectState
```
geometry_msgs/PoseStamped[] pose
```

The ObjectState message consists of:
- A list of PoseStamped messages called `pose` which specifies poses of any moving objects in the scene


__Example Usage:__
```python
my_pose = PoseStamped()
my_pose.header.frame_id = 'red_ring'
my_pose.position.x = 0.3
my_pose.position.y = 0.4
my_pose.position.z = -0.1
my_pose.orientation.x = 0.0
my_pose.orientation.y = 0.0
my_pose.orientation.z = 0.0
my_pose.orientation.y = 0.0

msg = ObjectState()
msg.pose = [my_pose]
```

<br>

## Services

The `teleop_interfaces` package features the following services:
- `Grasp`: Moves the hand to a specified object and grasps it
- `ExecuteTrajectory`: Moves the specified object in the trajectory it followed in Gazebo

### teleop_interfaces/Grasp
```
string object_id
---
bool success
```

A GraspRequest consists of:
- A string called `object_id` which specifies the object to be grasped


__Example Usage:__
```python
req = Grasp.Request()
req.object_id = 'green_ring'
```

<br>

### teleop_interfaces/ExecuteTrajectory
```
string object_id
---
bool success
```

An ExecuteTrajectoryRequest consists of:
- A string called `object_id` which specifies the object to be moved


__Example Usage:__
```python
req = ExecuteTrajectory.Request()
req.object_id = 'blue_ring'