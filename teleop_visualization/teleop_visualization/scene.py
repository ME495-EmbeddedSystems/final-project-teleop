"""
Publishes the avatar workspace.

Publishers:
  + avatar_right/joint_states (geometry_msgs/msg/JointState) - Joint states for the right avatar arm (ABB Gofa + Shadow Hand)
  + visualization_marker (visualization_msgs/msg/Marker) - Table marker

Subscribers:
  + avatar/right_arm/gofa2/joint_states (geometry_msgs/msg/JointState) - Joint states of the right ABB Gofa
  + joint_states (geometry_msgs/msg/JointState) - Joint states of the Shadow Hands

"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster



class Scene(Node):
    """
    Sets up table and robot in scene.
    """

    def __init__(self):
        super().__init__('scene')

        # Create control loop timer based on frequency parameter
        self.timer = self.create_timer(1/100, self.timer_callback)

        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Transform for abb base in world frame
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

        # Create table marker
        self.table = Marker()
        self.table.header.stamp = self.get_clock().now().to_msg()
        self.table.header.frame_id = 'world'
        self.table.id = 0
        self.table.type = self.table.CUBE
        self.table.action = self.table.ADD
        self.table.pose.position.z = -0.2655
        self.table.scale.x = 2.0
        self.table.scale.y = 2.0
        self.table.scale.z = 0.5
        self.table.color.r = 0.68
        self.table.color.g = 0.51
        self.table.color.b = 0.32
        self.table.color.a = 1.0
        self.table.lifetime.nanosec = 0
        self.table.frame_locked = True

        # Subscribe to ABB Gofa and Shadow Hand joint states
        self.gofa_js_subscription = self.create_subscription(JointState, 'avatar/right_arm/gofa2/joint_states', self.gofa_callback, 10)
        self.shadow_js_subscription = self.create_subscription(JointState, 'joint_states', self.shadow_callback, 10)

        # Publisher for avatar robot's joint states
        self.joint_pub = self.create_publisher(JointState, 'avatar_right/joint_states', 10)

        # Variables to hold latest joint states
        self.gofa_joint_names = ['gofa2_joint_1', 'gofa2_joint_2', 'gofa2_joint_3', 'gofa2_joint_4', 'gofa2_joint_5', 'gofa2_joint_6'] 
        self.gofa_js = [-1.570799, 0.174, 0.523, 0.0, -0.697, -0.523]
        self.shadow_joint_names = ['rh_FFJ4', 'rh_FFJ3','rh_FFJ2', 'rh_FFJ1','rh_MFJ4','rh_MFJ3','rh_MFJ2','rh_MFJ1','rh_RFJ4','rh_RFJ3','rh_RFJ2','rh_RFJ1','rh_LFJ5', 'rh_LFJ4','rh_LFJ3','rh_LFJ2','rh_LFJ1','rh_THJ5','rh_THJ4','rh_THJ3','rh_THJ2','rh_THJ1', 'rh_WRJ1', 'rh_WRJ2']
        self.shadow_js = 24*[0.0]
        
    def timer_callback(self):
        """Timer callback for Scene node."""
        # Create JointState object for avatar robot
        self.avatar_js = JointState()
        self.avatar_js.header.stamp = self.get_clock().now().to_msg()
        self.avatar_js.name = self.gofa_joint_names + self.shadow_joint_names
        self.avatar_js.position = self.gofa_js + self.shadow_js

        # Publish avatar robot joint states
        self.joint_pub.publish(self.avatar_js)

        # Publish table marker
        self.marker_pub.publish(self.table)

    def gofa_callback(self, msg):
        """
        Updates Gofa joint angles.

        Args
        ----
            msg (JointState): The current joint states of the right ABB Gofa

        """
        self.gofa_js = list(msg.position)

    def shadow_callback(self, msg):
        """
        Updates Shadow Hand joint angles.

        Args
        ----
            msg (JointState): The current joint states of the Shadow Hands

        """
        self.shadow_joint_names = []
        self.shadow_js = []

        for i in range(len(msg.name)):
            # If it's a joint for the right hand
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
