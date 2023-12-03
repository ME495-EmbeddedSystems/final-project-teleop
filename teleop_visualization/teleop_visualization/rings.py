"""
Publishes ring markers to reflect the positions of rings in the avatar workspace.

Publishers:
  + visualization_marker (visualization_msgs/msg/Marker) - Publish ring markers

"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer


class Rings(Node):
    """
    Sets up table and robot in scene.
    """

    def __init__(self):
        super().__init__('scene')

        # Create control loop timer based on frequency parameter
        self.timer = self.create_timer(1/100, self.timer_callback)

        # Create Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create publisher for markers
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, "visualization_marker", markerQoS)

        # Variables to keep track of ring's marker ID and start position
        self.ring_x = -0.3
        self.ring_id = 1
        
    def timer_callback(self):
        """Timer callback for Rings node."""

        # Update positions of the rings
        self.update_ring_positions()

        # Publish updated ring markers
        self.marker_pub.publish(self.blue_ring)
        self.marker_pub.publish(self.green_ring)
        self.marker_pub.publish(self.yellow_ring)
        self.marker_pub.publish(self.orange_ring)
        self.marker_pub.publish(self.red_ring)

    def generate_ring_marker(self, color):
        """
        Generates a Marker message to display a ring.

        Args
        ----
            color (string): The ring color

        Returns
        -------
           A visualization_msgs/Marker for the ring

        """
        # Maps color string to RGB values
        colors = {
            'blue': [0.0, 0.75, 0.86],
            'green': [0.65, 0.83, 0.02],
            'yellow': [0.94, 0.79, 0.16],
            'orange': [0.97, 0.48, 0.0],
            'red': [0.88, 0.0, 0.06],
        }

        # Maps color string to ring size/scale
        scales = {
            'blue': 1.0,
            'green': 0.93,
            'yellow': 0.86,
            'orange': 0.8,
            'red': 0.75,
        }

        # Create a populate the Marker message
        ring = Marker()
        ring.header.stamp = self.get_clock().now().to_msg()
        ring.header.frame_id = 'world'
        ring.id = self.ring_id
        ring.type = ring.MESH_RESOURCE
        ring.action = ring.ADD
        ring.pose.position.x = self.ring_x
        ring.pose.position.y = -0.5
        ring.pose.orientation.x = 0.707
        ring.pose.orientation.w = 0.707
        ring.scale.x = scales[color]
        ring.scale.y = 1.0
        ring.scale.z = scales[color]
        ring.color.r = colors[color][0]
        ring.color.g = colors[color][1]
        ring.color.b = colors[color][2]
        ring.color.a = 1.0
        ring.lifetime.nanosec = 0
        ring.frame_locked = True
        ring.mesh_resource = "package://teleop_visualization/ring.stl"

        # Increment the marker ID and initial x position for next ring marker
        self.ring_id += 1
        self.ring_x -= 0.15

        return ring
        
    def update_ring_positions(self):
        """Updates the ring marker positions."""
        # If the ring's transform exists, update the marker position
        if self.tf_buffer.can_transform('tag16H05_3', 'blue_center', rclpy.time.Time()):
            tf = self.tf_buffer.lookup_transform('tag16H05_3', 'blue_center', rclpy.time.Time()).transform
            self.blue_ring.pose.position.x = tf.translation.x
            self.blue_ring.pose.position.y = tf.translation.y
        if self.tf_buffer.can_transform('tag16H05_3', 'green_center', rclpy.time.Time()):
            tf = self.tf_buffer.lookup_transform('tag16H05_3', 'green_center', rclpy.time.Time()).transform
            self.green_ring.pose.position.x = tf.translation.x
            self.green_ring.pose.position.y = tf.translation.y
        if self.tf_buffer.can_transform('tag16H05_3', 'yellow_center', rclpy.time.Time()):
            tf = self.tf_buffer.lookup_transform('tag16H05_3', 'yellow_center', rclpy.time.Time()).transform
            self.yellow_ring.pose.position.x = tf.translation.x
            self.yellow_ring.pose.position.y = tf.translation.y
        if self.tf_buffer.can_transform('tag16H05_3', 'yellow_center', rclpy.time.Time()):
            tf = self.tf_buffer.lookup_transform('tag16H05_3', 'yellow_center', rclpy.time.Time()).transform
            self.orange_ring.pose.position.x = tf.translation.x
            self.orange_ring.pose.position.y = tf.translation.y
        if self.tf_buffer.can_transform('tag16H05_3', 'red_center', rclpy.time.Time()):
            tf = self.tf_buffer.lookup_transform('tag16H05_3', 'red_center', rclpy.time.Time()).transform
            self.red_ring.pose.position.x = tf.translation.x
            self.red_ring.pose.position.y = tf.translation.y
        
def main(args=None):
    rclpy.init(args=args)

    rings_node = Rings()

    rclpy.spin(rings_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rings_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
