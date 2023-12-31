"""
Set up arena and brick markers and moves brick.

Services:
  + grasp (teleop_interfaces/srv/Grasp) - Grasp a specific object
  + home (std_srvs/srv/Empty) - Move arm and hand to home configuration
  + release (std_srvs/srv/Empty) - Open hand
  + grasp_sequence (std_srvs/srv/Empty) - Stack green, yellow and orange ring
  + load_rings (std_srvs/srv/Empty) - Load positions of all rings
  + execute_trajectory (std_srvs/srv/ExecuteTrajectory) - Move grasped object along a trajectory

"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from teleop_interfaces.srv import Grasp, ExecuteTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import numpy as np
from tf_transformations import quaternion_from_matrix, quaternion_matrix
from std_srvs.srv import Empty
import time


class AvatarControl(Node):
    """Actuate the ABB Gofas and Shadow Hands."""

    def __init__(self):
        super().__init__("avatar_control")

        # Create timer
        timer_freq = 50  # Hz
        self.timer = self.create_timer(1 / timer_freq, self.timer_callback)

        # Create Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # PUBLISHERS

        # Shadow Hand publisher
        self.shadow_pub = self.create_publisher(
            JointTrajectory, "rh_trajectory_controller/command", 10
        )
        self.joint_names = [
            "rh_FFJ4",
            "rh_FFJ3",
            "rh_FFJ2",
            "rh_FFJ1",
            "rh_MFJ4",
            "rh_MFJ3",
            "rh_MFJ2",
            "rh_MFJ1",
            "rh_RFJ4",
            "rh_RFJ3",
            "rh_RFJ2",
            "rh_RFJ1",
            "rh_LFJ5",
            "rh_LFJ4",
            "rh_LFJ3",
            "rh_LFJ2",
            "rh_LFJ1",
            "rh_THJ5",
            "rh_THJ4",
            "rh_THJ3",
            "rh_THJ2",
            "rh_THJ1",
        ]

        # ABB Gofa Pose publisher
        self.abb_pub = self.create_publisher(
            TransformStamped, "/avatar/right_arm/target_tf", 10
        )

        # SERVICES

        # Create the /load_rings service
        self.load_rings = self.create_service(
            Empty, "load_rings", self.load_rings_callback
        )

        # Create the /release service
        self.release = self.create_service(Empty, "release", self.release_callback)

        # Create the /grasp service
        self.grasp = self.create_service(Grasp, "grasp", self.grasp_callback)

        # Create the /grasp_sequence service
        self.grasp_sequence = self.create_service(
            Empty, "grasp_sequence", self.grasp_sequence_callback
        )

        # Create the /home service
        self.home = self.create_service(Empty, "home", self.home_callback)

        # Create the /execute_trajectory service
        self.execute = self.create_service(
            ExecuteTrajectory, "execute_trajectory", self.execute_callback
        )

        # Transform of ring in hand frame
        self.T_hand_obj = np.array(
            [
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, 0.4695, 0.8829, -0.075],
                [0.0, 0.8829, -0.4695, 0.12],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        # Transform from avatar workspace to abb table april tag
        self.T_aws_world = np.array(
            [[1, 0, 0, 0.4], [0, 1, 0, -0.585], [0, 0, 1, 0.01], [0, 0, 0, 1]]
        )

        # Transform from abb_table april tag to peg
        self.T_world_peg = np.array(
            [
                [0.999, -0.033, -0.011, -0.234],
                [0.033, 0.999, 0.002, 0.491],
                [0.011, -0.002, 1.0, -0.021],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        # Transform of ring in hand frame
        self.T_hand_peg = np.array(
            [
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, 0.4695, 0.8829, -0.075],
                [0.0, 0.8829, -0.4695, 0.12],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        # Transform from avatar workspace to home
        self.T_aws_home = np.array(
            [[-1, 0, 0, 0.45], [0, 0, 1, -0.2], [0, 1, 0, 0.3], [0.0, 0.0, 0.0, 1.0]]
        )

        # Transform of ring of interest in world frame
        self.T_world_obj = np.eye(4)

        self.graspStandoff = 0.1 + 0.05

        self.object_frame_map = {
            "blue_ring": "blue_center",
            "green_ring": "green_center",
            "yellow_ring": "yellow_center",
            "orange_ring": "orange_center",
            "red_ring": "red_center",
        }

        self.world_ring_tf = {}

        self.buffer = {}

        initial_pose = TransformStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "operator_task_ws"
        initial_pose.child_frame_id = "haptxRight_sr_aligned"  # Right arm
        initial_pose.transform.translation.x = 0.5
        initial_pose.transform.translation.y = -0.063
        initial_pose.transform.translation.z = 0.385
        initial_pose.transform.rotation.x = 0.5075
        initial_pose.transform.rotation.y = 0.5293
        initial_pose.transform.rotation.z = 0.4705
        initial_pose.transform.rotation.w = -0.4908

        self.abb_pub.publish(initial_pose)

    def timer_callback(self):
        """Timer function for the Avatar Control node."""
        pass

    def load_rings_callback(self, request, response):
        # Store tf of all rings in a dictionary
        for object_id, object_frame in self.object_frame_map.items():
            try:
                self.world_ring_tf[object_id] = self.tf_buffer.lookup_transform(
                    "tag16H05_3", object_frame, rclpy.time.Time()
                )
                self.get_logger().info("Loading " + object_id)
            except Exception as e:
                self.get_logger().info("Could not load " + object_id)
                self.get_logger().info(f"{e}")

        return response

    def release_callback(self, request, response):
        """
        Open hand.

        Args:
            request (None) : Empty.

        Returns
        -------
            response (None) : Empty.

        """
        # Open hand
        self.shadow_pub.publish(self.open_hand_msg())
        self.get_logger().info("Releasing")
        time.sleep(25)

    def grasp_callback(self, request, response):
        """
        Grasp and stack a given ring.

        Args:
            request.object_id (str) : Empty.

        Returns
        -------
            response (None) : Empty.

        """
        # Open hand
        self.shadow_pub.publish(self.open_hand_msg())
        self.get_logger().info("Opening hand before motion")
        time.sleep(25)

        # Begin pick and place motion for specific object
        self.pick_and_place(request.object_id)

        return response

    def grasp_sequence_callback(self, request, response):
        """
        Grasp and stack green, yellow and orange rings in that order.

        Args:
            request (None) : Empty.

        Returns
        -------
            response (None) : Empty.

        """
        object_id_sequence = ["green_ring", "yellow_ring", "orange_ring"]

        # Open hand
        self.shadow_pub.publish(self.open_hand_msg())
        self.get_logger().info("Opening hand before motion sequence")
        time.sleep(25)

        # Begin pick and place sequence for listed object
        for object_id in object_id_sequence:
            self.pick_and_place(object_id)

        # Home configuration
        abb_msg = self.SE3_to_transform_stamped(self.T_aws_home)

        self.abb_pub.publish(abb_msg)
        self.get_logger().info("Going Home")
        time.sleep(5)

        return response

    def home_callback(self, request, response):
        """
        Return arm and hand to home configuration.

        Args:
            request (None) : Empty.

        Returns
        -------
            response (None) : Empty.

        """
        # Home configuration
        abb_msg = self.SE3_to_transform_stamped(self.T_aws_home)

        self.abb_pub.publish(abb_msg)
        self.get_logger().info("Going Home")
        time.sleep(5)

        # Open hand
        self.shadow_pub.publish(self.open_hand_msg())
        self.get_logger().info("Opening hand at Home")
        time.sleep(5)

        return response

    def execute_callback(self, request, response):
        """
        Grasp and stack green, yellow and orange rings in that order.

        Args:
            request (None) : Empty.

        Returns
        -------
            response (None) : Empty.

        """
        # Get necessary transforms
        world_peg_tf = self.tf_buffer.lookup_transform(
            "tag16H05_3", "tag16H05_10", rclpy.time.Time()
        )
        T_world_peg = self.transform_to_SE3(world_peg_tf)

        while len(self.buffer[request.object_id]) > 0:
            T_peg_obj = self.transform_to_SE3(self.buffer[request.object_id][0])

            # Calculate transformation of hand relative to avatar_ws
            T_aws_hand = (
                np.linalg.inv(self.T_world_aws)
                @ T_world_peg
                @ T_peg_obj
                @ np.linalg.inv(self.T_hand_obj)
            )

            abb_msg = self.SE3_to_transform_stamped(T_aws_hand)
            self.abb_pub.publish(abb_msg)

            del self.buffer[request.object_id][0]

        response.success = True

        return response

    def pick_and_place(self, object_id):
        """
        Grasp and stack any given ring.

        Args:
        ----
            object_id (str) : Local ID of the object to be stacked.

        """
        # Get necessary transform
        world_obj_tf = self.world_ring_tf[object_id]

        # Constrain object to be properly oriented on the table
        world_obj_tf.transform.translation.z = 0
        world_obj_tf.transform.rotation.x = 0.023263087438040456
        world_obj_tf.transform.rotation.y = -0.012624678671690372
        world_obj_tf.transform.rotation.z = -0.15317466259220655
        world_obj_tf.transform.rotation.w = 0.9878446077147206
        self.T_world_obj = self.transform_to_SE3(world_obj_tf)

        # Calculate transformation of hand relative to the avatar_ws for grabbing object
        T_aws_hand = (
            self.T_aws_world @ self.T_world_obj @ np.linalg.inv(self.T_hand_obj)
        )
        abb_msg = self.SE3_to_transform_stamped(T_aws_hand)

        # Move to standoff height above object
        abb_msg.transform.translation.z += self.graspStandoff
        abb_msg.transform.translation.x += 0.02
        self.abb_pub.publish(abb_msg)
        self.get_logger().info("Moving to standoff above " + object_id)
        time.sleep(8)

        # Move down to grab object
        abb_msg.transform.translation.z -= self.graspStandoff - 0.01
        self.abb_pub.publish(abb_msg)
        self.get_logger().info("Moving to grab object")
        time.sleep(8)

        # Close hand to grab object
        self.shadow_pub.publish(self.close_hand_msg())
        self.get_logger().info("Closing Hand")
        time.sleep(25)

        # Move to standoff height above object
        abb_msg.transform.translation.z += self.graspStandoff + 0.10
        self.abb_pub.publish(abb_msg)
        self.get_logger().info("Moving to standoff above " + object_id)
        if object_id == "orange_ring":
            time.sleep(15)
        time.sleep(3)

        # Calculate transformation of hand relative to the avatar_ws for stacking ring
        T_aws_hand = (
            self.T_aws_world @ self.T_world_peg @ np.linalg.inv(self.T_hand_peg)
        )
        abb_msg = self.SE3_to_transform_stamped(T_aws_hand)

        # Move to standoff above peg
        abb_msg.transform.translation.z += 0.3
        self.abb_pub.publish(abb_msg)
        self.get_logger().info("Moving to standoff above peg")

        # Move to drop position
        abb_msg.transform.translation.z -= 0.07

        self.abb_pub.publish(abb_msg)
        self.get_logger().info("Moving above peg")
        time.sleep(8)

        # Open hand
        self.shadow_pub.publish(self.open_hand_msg())
        self.get_logger().info("Placing " + object_id)
        time.sleep(25)

    def open_hand_msg(self):
        """
        Message to open the hand.

        Returns
        -------
            msg (trajectory_msgs/msg/JointTrajectory): Message to open hand.

        """
        point = JointTrajectoryPoint()
        point.positions = [
            0.027,
            0.091,
            0.0,
            0.0,
            0.147,
            0.098,
            0.0,
            0.0,
            -0.216,
            0.152,
            0.0,
            0.0,
            0.0,
            -0.349,
            0.024,
            0.248,
            0.0,
            0.1,
            1.2,
            0.05,
            0.15,
            -0.025,
        ]
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start.nanosec = 100000000  # 50000000

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [point]

        return msg

    def close_hand_msg(self):
        """
        Message to close the hand.

        Returns
        -------
            msg (trajectory_msgs/msg/JointTrajectory): Message to close hand.

        """
        point = JointTrajectoryPoint()
        point.positions = [
            0.027,
            0.091,
            0.148,
            0.88,
            0.147,
            0.098,
            0.148,
            0.88,
            -0.216,
            0.152,
            0.148,
            0.88,
            0.0,
            -0.349,
            0.024,
            0.148,
            0.88,
            0.3,
            1.2,
            0.05,
            0.15,
            -0.025,
        ]
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start.nanosec = 100000000  # 50000000

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [point]

        return msg

    def transform_to_SE3(self, transform):
        """
        Convert a geometry_msgs/TransformStamped message to an SE(3) transformation matrix.

        Args:
            transform (geometry_msgs/TransformStamped): A transform object

        Returns
        -------
            A numpy array representing the transform as a 4x4 SE(3) matrix

        """
        x = transform.transform.rotation.x
        y = transform.transform.rotation.y
        z = transform.transform.rotation.z
        w = transform.transform.rotation.w

        T = np.array(quaternion_matrix([x, y, z, w]))
        T[0, 3] = transform.transform.translation.x
        T[1, 3] = transform.transform.translation.y
        T[2, 3] = transform.transform.translation.z

        return T

    def SE3_to_transform_stamped(self, T):
        """
        Convert an SE(3) transformation matrix to a geometry_msgs/TransformStamped message.

        Args:
            T (numpy array): A 4x4 SE(3) transformation matrix

        Returns
        -------
            A geometry_msgs/TransformStamped message to move the right ABB arm

        """
        quaternion_array = quaternion_from_matrix(T)

        quat = Quaternion()
        quat.x = quaternion_array[0]
        quat.y = quaternion_array[1]
        quat.z = quaternion_array[2]
        quat.w = quaternion_array[3]

        # Convert transformation matrix to stamped pose data
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "operator_task_ws"
        tf.child_frame_id = "haptxRight_sr_aligned"  # Right arm
        tf.transform.translation.x = T[0, 3]
        tf.transform.translation.y = T[1, 3]
        tf.transform.translation.z = T[2, 3]
        tf.transform.rotation = quat

        return tf


def main(args=None):
    rclpy.init(args=args)

    avatar_control_node = AvatarControl()

    rclpy.spin(avatar_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avatar_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
