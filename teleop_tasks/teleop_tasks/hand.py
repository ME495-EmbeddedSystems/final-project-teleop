"""Broadcasts fixed frames to be used by the simulated world."""
import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import (
    TransformStamped, TransformBroadcaster)
from tf2_ros.buffer import Buffer


class Objects(Node):
    """Creates a node of type object."""

    def __init__(self):
        super().__init__("objects")
        # Creating a timer for this node
        self.timer = self.create_timer(1 / 100, self.timer_callback)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def publishWorld(self):
        """Publish the fixed frames needed for the sim."""
        # Need to braodcast world so right and left hands have a fixed frame
        lh_world = TransformStamped()

        lh_world.header.stamp = self.get_clock().now().to_msg()
        lh_world.header.frame_id = "sim/world"
        lh_world.child_frame_id = "left_hand/world"
        lh_world.transform.translation.x = 0.0
        lh_world.transform.translation.y = 0.0
        lh_world.transform.translation.z = 0.0
        lh_world.transform.rotation.x = 0.0
        lh_world.transform.rotation.y = 0.0
        lh_world.transform.rotation.z = 0.0
        lh_world.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(lh_world)

        rh_world = TransformStamped()

        rh_world.header.stamp = self.get_clock().now().to_msg()
        rh_world.header.frame_id = "sim/world"
        rh_world.child_frame_id = "right_hand/world"
        rh_world.transform.translation.x = 0.0
        rh_world.transform.translation.y = 0.0
        rh_world.transform.translation.z = 0.0
        rh_world.transform.rotation.x = 0.0
        rh_world.transform.rotation.y = 0.0
        rh_world.transform.rotation.z = 0.0
        rh_world.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(rh_world)

    def timer_callback(self):
        """Timer callback for this node."""
        self.publishWorld()


def main(args=None):
    """Objects' main function."""
    rclpy.init(args=args)
    node = Objects()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
