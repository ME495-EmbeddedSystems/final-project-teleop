import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from enum import Enum
import numpy as np
import random
import time

class SimState(Enum):
    REST = 0
    GRASPED = 1
    FALLING = 2

class RingSim(Node):
    """
    Publishes transforms for all the rings.
    """

    def __init__(self):
        super().__init__('ring_sim')

        # Create control loop timer based on frequency parameter
        self.timer = self.create_timer(1/100, self.timer_callback)

        # Create broadcasters
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Create Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ring_positions = [(0,0)]

        # Transform for blue ring in world
        self.world_to_blue = TransformStamped()
        self.world_to_blue.header.frame_id = "sim/world"
        self.world_to_blue.child_frame_id = "sim/blue/center"
        x, y = self.generate_ring_start_position()
        self.world_to_blue.transform.translation.x = x
        self.world_to_blue.transform.translation.y = y
        self.world_to_blue.transform.translation.z = 0.0155

        # Transform for green ring in world
        self.world_to_green = TransformStamped()
        self.world_to_green.header.frame_id = "sim/world"
        self.world_to_green.child_frame_id = "sim/green/center"
        x, y = self.generate_ring_start_position()
        self.world_to_green.transform.translation.x = x
        self.world_to_green.transform.translation.y = y
        self.world_to_green.transform.translation.z = 0.0155

        # Transform for yellow ring in world
        self.world_to_yellow = TransformStamped()
        self.world_to_yellow.header.frame_id = "sim/world"
        self.world_to_yellow.child_frame_id = "sim/yellow/center"
        x, y = self.generate_ring_start_position()
        self.world_to_yellow.transform.translation.x = x
        self.world_to_yellow.transform.translation.y = y
        self.world_to_yellow.transform.translation.z = 0.0155

        # Transform for orange ring in world
        self.world_to_orange = TransformStamped()
        self.world_to_orange.header.frame_id = "sim/world"
        self.world_to_orange.child_frame_id = "sim/orange/center"
        x, y = self.generate_ring_start_position()
        self.world_to_orange.transform.translation.x = x
        self.world_to_orange.transform.translation.y = y
        self.world_to_orange.transform.translation.z = 0.0155

        # Transform for red ring in world
        self.world_to_red = TransformStamped()
        self.world_to_red.header.frame_id = "sim/world"
        self.world_to_red.child_frame_id = "sim/red/center"
        x, y = self.generate_ring_start_position()
        self.world_to_red.transform.translation.x = x
        self.world_to_red.transform.translation.y = y
        self.world_to_red.transform.translation.z = 0.0155

        # Transform for red ring in world
        self.world_to_ring_base = TransformStamped()
        self.world_to_ring_base.header.frame_id = "sim/world"
        self.world_to_ring_base.child_frame_id = "sim/ring_base/base"

        # Create publisher for markers
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, "visualization_marker", markerQoS)

        # Publish table marker
        table = Marker()
        table.header.stamp = self.get_clock().now().to_msg()
        table.header.frame_id = 'sim/world'
        table.type = table.CUBE
        table.action = table.ADD
        table.pose.position.z = -0.25
        table.scale.x = 2.0
        table.scale.y = 2.0
        table.scale.z = 0.5
        table.color.r = 0.68
        table.color.g = 0.51
        table.color.b = 0.32
        table.color.a = 1.0
        table.lifetime.nanosec = 0
        table.frame_locked = True
        table.id = 0
        self.marker_pub.publish(table)

        self.object_frame_map = {'blue_ring': 'blue_center',
                                 'green_ring': 'green_center',
                                 'yellow_ring': 'yellow_center',
                                 'orange_ring': 'orange_center',
                                 'red_ring': 'red_center'}
        
        self.state = SimState.REST
        
    def timer_callback(self):
        self.publish_world()

        self.get_logger().info(str(self.state))

        if self.state == SimState.REST:
            # If ring has been grasped, transition to GRASPED state
            if self.ring_grasped():
                self.state = SimState.GRASPED

        if self.state == SimState.GRASPED:
            # Publish Ring's TFs based on hand position
            
            # IF OBJECT HAS BEEN RELEASED
            self.state == SimState.FALLING

        if self.state == SimState.FALLING:
            # IF OBJECT HAS HIT PEG BASE (0.125 m x 0.125 m x 0.028 m) or IF OBJECT HAS HIT GROUND
            self.state = SimState.REST

            # Otherwise, make brick fall

    def publish_world(self):
        self.world_to_blue.header.stamp = self.get_clock().now().to_msg()
        self.world_to_green.header.stamp = self.get_clock().now().to_msg()
        self.world_to_yellow.header.stamp = self.get_clock().now().to_msg()
        self.world_to_orange.header.stamp = self.get_clock().now().to_msg()
        self.world_to_red.header.stamp = self.get_clock().now().to_msg()

        self.broadcaster.sendTransform(self.world_to_blue)
        self.broadcaster.sendTransform(self.world_to_green)
        self.broadcaster.sendTransform(self.world_to_yellow)
        self.broadcaster.sendTransform(self.world_to_orange)
        self.broadcaster.sendTransform(self.world_to_red)
        self.broadcaster.sendTransform(self.world_to_ring_base)

    def ring_grasped(self):
        """ Calculates 1x5 bitmap of which fingers are in contact with the ring.
            If thumb and another finger are touching a ring, that ring is grabbed.
        """

        try:
            # Lookup tf from ring to fingertips
            yellow_thumb_tf = self.tf_buffer.lookup_transform("sim/yellow/center", "left_hand/thumb_tip", rclpy.time.Time()) # Thumb
            yellow_index_tf = self.tf_buffer.lookup_transform("sim/yellow/center", "left_hand/index_tip", rclpy.time.Time()) # Index
            yellow_middle_tf = self.tf_buffer.lookup_transform("sim/yellow/center", "left_hand/middle_tip", rclpy.time.Time()) # Middle
            yellow_ring_tf = self.tf_buffer.lookup_transform("sim/yellow/center", "left_hand/ring_tip", rclpy.time.Time()) # Ring
            yellow_pinky_tf = self.tf_buffer.lookup_transform("sim/yellow/center", "left_hand/pinky_tip", rclpy.time.Time()) # Pinky

            fingers_in_contact = np.zeros(5)

            fingers_in_contact[0] = self.finger_contact(yellow_thumb_tf)
            fingers_in_contact[1] = self.finger_contact(yellow_index_tf)
            fingers_in_contact[2] = self.finger_contact(yellow_middle_tf)
            fingers_in_contact[3] = self.finger_contact(yellow_ring_tf)
            fingers_in_contact[4] = self.finger_contact(yellow_pinky_tf)

            self.get_logger().info(str(np.sum(fingers_in_contact)))

            if (fingers_in_contact[0] == 1) and (np.sum(fingers_in_contact) >= 2):

                return True
        
        except Exception as error:
            self.get_logger().info(str(error))

        return False


    def finger_contact(self, ring_fingertip_tf):

        # Ring Dimensions
        ring_width = 0.118
        ring_height = 0.031

        # Check if z is within height range of ring
        if abs(ring_fingertip_tf.transform.translation.z) <= ring_height/2:

            dist_fingertip_ring = (ring_fingertip_tf.transform.translation.x**2 + ring_fingertip_tf.transform.translation.y**2)**0.5

            if dist_fingertip_ring <= ring_width/2:

                return 1
            
        return 0

    def generate_ring_start_position(self):
        x = random.uniform(-0.5, 0.5)
        y = random.uniform(-0.5, 0.5)

        while self.too_close(x,y):
            x = random.uniform(-0.5, 0.5)
            y = random.uniform(-0.5, 0.5)

        self.ring_positions += [(x,y)]

        return x, y

    def too_close(self, x, y):
        for pos in self.ring_positions:
            d = ((x - pos[0])**2 + (y - pos[1])**2)**0.5

            if d < 0.15:
                return True

        return False
    
def main(args=None):
    rclpy.init(args=args)

    ring_sim_node = RingSim()

    rclpy.spin(ring_sim_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ring_sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
