import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer



class RingsLight(Node):
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

        self.ring_x = -0.3
        self.ring_id = 1

        # Publish rings
        self.blue_ring = self.generate_ring_marker('blue')
        self.green_ring = self.generate_ring_marker('green')
        self.yellow_ring = self.generate_ring_marker('yellow')
        self.orange_ring = self.generate_ring_marker('orange')
        self.red_ring = self.generate_ring_marker('red')

        self.marker_pub.publish(self.blue_ring)
        self.marker_pub.publish(self.green_ring)
        self.marker_pub.publish(self.yellow_ring)
        self.marker_pub.publish(self.orange_ring)
        self.marker_pub.publish(self.red_ring)

        self.object_frame_map = {'blue_ring': 'blue_center',
                                 'green_ring': 'green_center',
                                 'yellow_ring': 'yellow_center',
                                 'orange_ring': 'orange_center',
                                 'red_ring': 'red_center'}
        
    def timer_callback(self):
        self.update_ring_positions()

    def generate_ring_marker(self, color):

        colors = {
            'blue': [0.0, 0.75, 0.86],
            'green': [0.65, 0.83, 0.02],
            'yellow': [0.94, 0.79, 0.16],
            'orange': [0.97, 0.48, 0.0],
            'red': [0.88, 0.0, 0.06],
        }

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
        ring.scale.x = 1.0
        ring.scale.y = 1.0
        ring.scale.z = 1.0
        ring.color.r = colors[color][0]
        ring.color.g = colors[color][1]
        ring.color.b = colors[color][2]
        ring.color.a = 1.0
        ring.lifetime.nanosec = 0
        ring.frame_locked = True
        ring.mesh_resource = "package://teleop_visualization/ring.stl"

        self.ring_id += 1
        self.ring_x -= 0.15

        return ring
        
    def update_ring_positions(self):
        # If the ring's transform exists, update it
        if self.tf_buffer.can_transform('tag16H05_3', self.object_frame_map['blue_ring'], rclpy.time.Time()):
            tf = self.tf_buffer.lookup_transform('tag16H05_3', self.object_frame_map['blue_ring'], rclpy.time.Time()).transform
            self.blue_ring.pose.position.x = tf.translation.x
            self.blue_ring.pose.position.y = tf.translation.y
        if self.tf_buffer.can_transform('tag16H05_3', self.object_frame_map['green_ring'], rclpy.time.Time()):
            tf = self.tf_buffer.lookup_transform('tag16H05_3', self.object_frame_map['green_ring'], rclpy.time.Time()).transform
            self.green_ring.pose.position.x = tf.translation.x
            self.green_ring.pose.position.y = tf.translation.y
        if self.tf_buffer.can_transform('tag16H05_3', self.object_frame_map['yellow_ring'], rclpy.time.Time()):
            tf = self.tf_buffer.lookup_transform('tag16H05_3', self.object_frame_map['yellow_ring'], rclpy.time.Time()).transform
            self.yellow_ring.pose.position.x = tf.translation.x
            self.yellow_ring.pose.position.y = tf.translation.y
        if self.tf_buffer.can_transform('tag16H05_3', self.object_frame_map['orange_ring'], rclpy.time.Time()):
            tf = self.tf_buffer.lookup_transform('tag16H05_3', self.object_frame_map['orange_ring'], rclpy.time.Time()).transform
            self.orange_ring.pose.position.x = tf.translation.x
            self.orange_ring.pose.position.y = tf.translation.y
        if self.tf_buffer.can_transform('tag16H05_3', self.object_frame_map['red_ring'], rclpy.time.Time()):
            tf = self.tf_buffer.lookup_transform('tag16H05_3', self.object_frame_map['red_ring'], rclpy.time.Time()).transform
            self.red_ring.pose.position.x = tf.translation.x
            self.red_ring.pose.position.y = tf.translation.y


def main(args=None):
    rclpy.init(args=args)

    rings_light_node = RingsLight()

    rclpy.spin(rings_light_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rings_light_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
