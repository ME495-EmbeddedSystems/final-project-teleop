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

        # Create broadcasters
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Create Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        time.sleep(3)

        # Transform for blue ring in world
        self.world_to_blue = TransformStamped()
        self.world_to_blue.header.frame_id = "world"
        self.world_to_blue.child_frame_id = "blue/center"
        # self.static_broadcaster.sendTransform(world_to_blue)

        time.sleep(0.5)

        # Transform for green ring in world
        self.world_to_green = TransformStamped()
        self.world_to_green.header.frame_id = "world"
        self.world_to_green.child_frame_id = "green/center"
        self.world_to_green.transform.translation.x = 0.2
        self.world_to_green.transform.translation.y = 0.3
        # self.static_broadcaster.sendTransform(world_to_green)

        time.sleep(0.5)

        # Transform for yellow ring in world
        self.world_to_yellow = TransformStamped()
        self.world_to_yellow.header.frame_id = "world"
        self.world_to_yellow.child_frame_id = "yellow/center"
        self.world_to_yellow.transform.translation.x = -0.1
        self.world_to_yellow.transform.translation.y = 0.15
        # self.broadcaster.sendTransform(self.world_to_yellow)

        time.sleep(0.5)

        # Transform for orange ring in world
        self.world_to_orange = TransformStamped()
        self.world_to_orange.header.frame_id = "world"
        self.world_to_orange.child_frame_id = "orange/center"
        self.world_to_orange.transform.translation.x = 0.1
        self.world_to_orange.transform.translation.y = -0.3
        # self.static_broadcaster.sendTransform(world_to_orange)

        time.sleep(0.5)

        # Transform for red ring in world
        self.world_to_red = TransformStamped()
        self.world_to_red.header.frame_id = "world"
        self.world_to_red.child_frame_id = "red/center"
        self.world_to_red.transform.translation.x = -0.05
        self.world_to_red.transform.translation.y = -0.2
        # self.static_broadcaster.sendTransform(world_to_red)

        time.sleep(0.5)

        # Transform for abb base in world
        world_to_abb = TransformStamped()
        world_to_abb.header.frame_id = "world"
        world_to_abb.child_frame_id = "avatar_right/gofa2_base"
        world_to_abb.transform.translation.x = -0.064
        world_to_abb.transform.translation.y = -0.385
        world_to_abb.transform.translation.z = 0.073
        world_to_abb.transform.rotation.x = 0.2588190714169043
        world_to_abb.transform.rotation.y = -0.00020610460516567008
        world_to_abb.transform.rotation.z = 0.9659254909850248
        world_to_abb.transform.rotation.w = 0.0007691925129359764
        self.static_broadcaster.sendTransform(world_to_abb)

        self.joint_pub = self.create_publisher(JointState, 'avatar_right/joint_states', 10)

        # Create control loop timer based on frequency parameter
        self.timer = self.create_timer(1/100, self.timer_callback)

        self.gofa_joint_names = ['gofa2_joint_1', 'gofa2_joint_2', 'gofa2_joint_3', 'gofa2_joint_4', 'gofa2_joint_5', 'gofa2_joint_6'] 
        self.gofa_js = [-1.570799, 0.174, 0.523, 0.0, -0.697, -0.523]
        self.shadow_joint_names = ['rh_FFJ4', 'rh_FFJ3','rh_FFJ2', 'rh_FFJ1','rh_MFJ4','rh_MFJ3','rh_MFJ2','rh_MFJ1','rh_RFJ4','rh_RFJ3','rh_RFJ2','rh_RFJ1','rh_LFJ5', 'rh_LFJ4','rh_LFJ3','rh_LFJ2','rh_LFJ1','rh_THJ5','rh_THJ4','rh_THJ3','rh_THJ2','rh_THJ1', 'rh_WRJ1', 'rh_WRJ2']
        self.shadow_js = 24*[0.0]

        self.avatar_js = JointState()
        # self.gofa_js.position = [-1.570799, 0.174, 0.523, 0.0, -0.697, -0.523] + [0.027, 0.091, 0.148, 0.88, 0.147, 0.098, 0.148, 0.88, -0.216, 0.152, 0.148, 0.88, 0.0, -0.349, 0.024, 0.148, 0.88, 0.3, 1.2, 0.05, 0.15, -0.025, 0.0, 0.0]

        self.gofa_js_subscription = self.create_subscription(JointState, 'avatar/right_arm/gofa2/joint_states', self.gofa_callback, 10)
        self.shadow_js_subscription = self.create_subscription(JointState, 'joint_states', self.shadow_callback, 10)
        
    def timer_callback(self):
        self.avatar_js.header.stamp = self.get_clock().now().to_msg()
        self.avatar_js.name = self.gofa_joint_names + self.shadow_joint_names
        self.avatar_js.position = self.gofa_js + self.shadow_js
        self.joint_pub.publish(self.avatar_js)

        if self.tf_buffer.can_transform('tag16H05_3', 'tag16H05_2', rclpy.time.Time()):
            self.world_to_yellow.transform = self.tf_buffer.lookup_transform('tag16H05_3', 'tag16H05_2', rclpy.time.Time()).transform
            self.broadcaster.sendTransform(self.world_to_yellow)

        self.broadcaster.sendTransform(self.world_to_blue)
        self.broadcaster.sendTransform(self.world_to_green)
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
