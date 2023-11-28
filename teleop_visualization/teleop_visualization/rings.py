import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import time


class Rings(Node):
    """
    Publishes transforms for all the rings.
    """

    def __init__(self):
        super().__init__('rings')

        # Create control loop timer based on frequency parameter
        self.timer = self.create_timer(1/100, self.timer_callback)

        # Create broadcasters
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Create Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Transform for blue ring in world
        self.world_to_blue = TransformStamped()
        self.world_to_blue.header.frame_id = "world"
        self.world_to_blue.child_frame_id = "blue/center"
        self.world_to_blue.transform.translation.x = -0.3
        self.world_to_blue.transform.translation.y = -0.5

        # Transform for green ring in world
        self.world_to_green = TransformStamped()
        self.world_to_green.header.frame_id = "world"
        self.world_to_green.child_frame_id = "green/center"
        self.world_to_green.transform.translation.x = -0.45
        self.world_to_green.transform.translation.y = -0.5

        # Transform for yellow ring in world
        self.world_to_yellow = TransformStamped()
        self.world_to_yellow.header.frame_id = "world"
        self.world_to_yellow.child_frame_id = "yellow/center"
        self.world_to_yellow.transform.translation.x = -0.6
        self.world_to_yellow.transform.translation.y = -0.5

        # Transform for orange ring in world
        self.world_to_orange = TransformStamped()
        self.world_to_orange.header.frame_id = "world"
        self.world_to_orange.child_frame_id = "orange/center"
        self.world_to_orange.transform.translation.x = -0.75
        self.world_to_orange.transform.translation.y = -0.5

        # Transform for red ring in world
        self.world_to_red = TransformStamped()
        self.world_to_red.header.frame_id = "world"
        self.world_to_red.child_frame_id = "red/center"
        self.world_to_red.transform.translation.x = -0.9
        self.world_to_red.transform.translation.y = -0.5

        self.object_frame_map = {'blue_ring': 'tag16H05_2',
                                 'green_ring': 'tag16H05_2',
                                 'yellow_ring': 'tag16H05_2',
                                 'orange_ring': 'tag16H05_2',
                                 'red_ring': 'tag16H05_2'}
        
    def timer_callback(self):
        # If the ring's transform exists, update it
        if self.tf_buffer.can_transform('tag16H05_3', self.object_frame_map['blue_ring'], rclpy.time.Time()):
            self.world_to_blue.transform = self.tf_buffer.lookup_transform('tag16H05_3', self.object_frame_map['blue_ring'], rclpy.time.Time()).transform
        if self.tf_buffer.can_transform('tag16H05_3', self.object_frame_map['green_ring'], rclpy.time.Time()):
            self.world_to_green.transform = self.tf_buffer.lookup_transform('tag16H05_3', self.object_frame_map['green_ring'], rclpy.time.Time()).transform
        if self.tf_buffer.can_transform('tag16H05_3', self.object_frame_map['yellow_ring'], rclpy.time.Time()):
            self.world_to_yellow.transform = self.tf_buffer.lookup_transform('tag16H05_3', self.object_frame_map['yellow_ring'], rclpy.time.Time()).transform
        if self.tf_buffer.can_transform('tag16H05_3', self.object_frame_map['orange_ring'], rclpy.time.Time()):
            self.world_to_orange.transform = self.tf_buffer.lookup_transform('tag16H05_3', self.object_frame_map['orange_ring'], rclpy.time.Time()).transform
        if self.tf_buffer.can_transform('tag16H05_3', self.object_frame_map['red_ring'], rclpy.time.Time()):
            self.world_to_red.transform = self.tf_buffer.lookup_transform('tag16H05_3', self.object_frame_map['red_ring'], rclpy.time.Time()).transform
            
        self.broadcaster.sendTransform(self.world_to_blue)
        self.broadcaster.sendTransform(self.world_to_green)
        self.broadcaster.sendTransform(self.world_to_yellow)
        self.broadcaster.sendTransform(self.world_to_orange)
        self.broadcaster.sendTransform(self.world_to_red)


    def gofa_callback(self, msg):
        self.gofa_js = list(msg.position)

    def shadow_callback(self, msg):

        self.shadow_joint_names = []
        self.shadow_js = []

        for i in range(len(msg.name)):
            if msg.name[i][0] == 'r':
                self.shadow_joint_names += [msg.name[i]]
                self.shadow_js += [msg.position[i]]


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
