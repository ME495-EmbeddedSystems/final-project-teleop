# `teleop_avatar` package

The `teleop_avatar` package controls the avatar consisting of an [ABB Gofa arm](https://new.abb.com/products/robotics/robots/collaborative-robots/crb-15000) and [Shadow Robot: Dexterous Hand](https://www.shadowrobot.com/dexterous-hand-series/), to perform a sequence of elementary actions, for accomplishing the task of stacking rings on a peg:
1. __Traveling to the standoff pose above a ring:__ Obtained by constraining the ring's pose (retrieved by `teleop_sensing`) to the workspace table.

2. __Moving down and grasping the ring:__ By fixing the transform between the hand and the ring during grasping, and closing the fingers around the object.

3. __Picking the ring up and carrying it above the peg:__ By knowing the fixed pose of the peg in the world, while avoiding collisions with it.

4. __Dropping the ring on the peg:__ By opening the fingers, and beginning the grasping motions for the next ring.

<br>

### Run The Avatar Controller

To run the avatar controller, you can use the following command:

```bash
ros2 run teleop_avatar avatar_control
```

<br>

You can now call the following services to make the `avatar_control` node stack rings on the peg:

1. `/home` service:

    ```bash
    ros2 run teleop_avatar avatar_control
    ```
    This service moves the arm to the home position and opens the hand such that the avatar is not blocking the camera's view of the objects and AprilTag, while ensuring that the hand is ready to grasp objects.

2. `/load_rings` service:

    This service needs to be called after the `rings_cv.launch.xml` file of the `teleop_sensing` package is launched. 

    ```bash
    ros2 service call /load_rings std_srvs/srv/Empty
    ```

    This service looks up the positions of the centers of each ring and records them within the node.

3. Now, you can either use the `/grasp` service:

    This service is of the `Grasp` type from the `teleop_interfaces` package, where the `object_id` request can be either one of:
    - `"blue_ring"`
    - `"green_ring"`
    - `"yellow_ring"`
    - `"orange_ring"`
    - `"red_ring"`

    An example is:
    ```bash
    ros2 service call /grasp teleop_interfaces/srv/Grasp "object_id: 'orange_ring'"
    ```

    This service commands the avatar to pick up a specific ring and place it on the peg.

4. or the `/grasp_sequence` service:

    ```bash
    ros2 service call /grasp_sequence std_srvs/srv/Empty
    ```

    This service by default commands the avatar to sequentially stack the green, yellow and orange rings on the peg.


