import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import random
import time


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
        self.world_to_blue.header.frame_id = "world"
        self.world_to_blue.child_frame_id = "sim/blue/center"
        x, y = self.generate_ring_start_position()
        self.world_to_blue.transform.translation.x = x
        self.world_to_blue.transform.translation.y = y
        self.world_to_blue.transform.translation.z = 0.0155
        self.broadcaster.sendTransform(self.world_to_blue)

        # Transform for green ring in world
        self.world_to_green = TransformStamped()
        self.world_to_green.header.frame_id = "world"
        self.world_to_green.child_frame_id = "sim/green/center"
        x, y = self.generate_ring_start_position()
        self.world_to_green.transform.translation.x = x
        self.world_to_green.transform.translation.y = y
        self.world_to_green.transform.translation.z = 0.0155
        self.broadcaster.sendTransform(self.world_to_green)

        # Transform for yellow ring in world
        self.world_to_yellow = TransformStamped()
        self.world_to_yellow.header.frame_id = "world"
        self.world_to_yellow.child_frame_id = "sim/yellow/center"
        x, y = self.generate_ring_start_position()
        self.world_to_yellow.transform.translation.x = x
        self.world_to_yellow.transform.translation.y = y
        self.world_to_yellow.transform.translation.z = 0.0155
        self.broadcaster.sendTransform(self.world_to_yellow)

        # Transform for orange ring in world
        self.world_to_orange = TransformStamped()
        self.world_to_orange.header.frame_id = "world"
        self.world_to_orange.child_frame_id = "sim/orange/center"
        x, y = self.generate_ring_start_position()
        self.world_to_orange.transform.translation.x = x
        self.world_to_orange.transform.translation.y = y
        self.world_to_orange.transform.translation.z = 0.0155
        self.broadcaster.sendTransform(self.world_to_orange)

        # Transform for red ring in world
        self.world_to_red = TransformStamped()
        self.world_to_red.header.frame_id = "world"
        self.world_to_red.child_frame_id = "sim/red/center"
        x, y = self.generate_ring_start_position()
        self.world_to_red.transform.translation.x = x
        self.world_to_red.transform.translation.y = y
        self.world_to_red.transform.translation.z = 0.0155
        self.broadcaster.sendTransform(self.world_to_red)

        # Transform for red ring in world
        self.world_to_ring_base = TransformStamped()
        self.world_to_ring_base.header.frame_id = "world"
        self.world_to_ring_base.child_frame_id = "sim/ring_base/base"
        self.static_broadcaster.sendTransform(self.world_to_ring_base)

        # Create publisher for markers
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, "visualization_marker", markerQoS)

        # Publish table marker
        table = Marker()
        table.header.stamp = self.get_clock().now().to_msg()
        table.header.frame_id = 'world'
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
        
    def timer_callback(self):
        pass

    def generate_ring_start_position(self):
        x = random.uniform(-0.5, 0.5)
        y = random.uniform(-0.5, 0.5)

        while self.too_close(x,y):
            x = random.uniform(-0.5, 0.5)
            y = random.uniform(-0.5, 0.5)

        # while (x > -0.15 and x < 0.1):
        #     x = random.uniform(-0.5, 0.5)

        # while (y > -0.15 and y < 0.15):
        #     y = random.uniform(-0.5, 0.5)

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
