import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer



class Scene(Node):
    """
    Sets up table and robot in scene.
    """

    def __init__(self):
        super().__init__('scene')

        # Create control loop timer based on frequency parameter
        self.timer = self.create_timer(1/100, self.timer_callback)

        # Create broadcasters
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Create Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        # Create publisher for markers
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, "visualization_marker", markerQoS)

        # Publish table marker
        table = Marker()
        table.header.stamp = self.get_clock().now().to_msg()
        table.header.frame_id = 'world'
        table.id = 0
        table.type = table.CUBE
        table.action = table.ADD
        table.pose.position.z = -0.2655
        table.scale.x = 2.0
        table.scale.y = 2.0
        table.scale.z = 0.5
        table.color.r = 0.68
        table.color.g = 0.51
        table.color.b = 0.32
        table.color.a = 1.0
        table.lifetime.nanosec = 0
        table.frame_locked = True
        self.marker_pub.publish(table)

        # Subscribe to avatar joint states
        self.gofa_js_subscription = self.create_subscription(JointState, 'avatar/right_arm/gofa2/joint_states', self.gofa_callback, 10)
        self.shadow_js_subscription = self.create_subscription(JointState, 'joint_states', self.shadow_callback, 10)

        # Publish joint states for avatar robot
        self.joint_pub = self.create_publisher(JointState, 'avatar_right/joint_states', 10)

        # Variables to hold latest joint states
        self.gofa_joint_names = ['gofa2_joint_1', 'gofa2_joint_2', 'gofa2_joint_3', 'gofa2_joint_4', 'gofa2_joint_5', 'gofa2_joint_6'] 
        self.gofa_js = [-1.570799, 0.174, 0.523, 0.0, -0.697, -0.523]
        self.shadow_joint_names = ['rh_FFJ4', 'rh_FFJ3','rh_FFJ2', 'rh_FFJ1','rh_MFJ4','rh_MFJ3','rh_MFJ2','rh_MFJ1','rh_RFJ4','rh_RFJ3','rh_RFJ2','rh_RFJ1','rh_LFJ5', 'rh_LFJ4','rh_LFJ3','rh_LFJ2','rh_LFJ1','rh_THJ5','rh_THJ4','rh_THJ3','rh_THJ2','rh_THJ1', 'rh_WRJ1', 'rh_WRJ2']
        self.shadow_js = 24*[0.0]
        
    def timer_callback(self):
        self.avatar_js = JointState()
        self.avatar_js.header.stamp = self.get_clock().now().to_msg()
        self.avatar_js.name = self.gofa_joint_names + self.shadow_joint_names
        self.avatar_js.position = self.gofa_js + self.shadow_js
        self.joint_pub.publish(self.avatar_js)

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

    scene_node = Scene()

    rclpy.spin(scene_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scene_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
