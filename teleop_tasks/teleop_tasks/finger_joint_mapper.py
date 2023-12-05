"""
Map the joint states from the haptx driver.

Reads in joint states from /haptx/lh/raw_joint_states and maps them to
/left_hand/joint_states

SUBSCRIBERS:
    /haptx/lh/raw_joint_states: sensor/msgs/JointState

PUBLISHERS:
    /left_hand/joint_states: sensor/msgs/JointState
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Mapper(Node):
    """Maps joint angles from HaptX driver to Gazebo."""

    def __init__(self):
        super().__init__("mapper")

        # Subscribe to finger angles of HaptX Glove
        self.haptx_sub = self.create_subscription(
            JointState, "/haptx/lh/raw_joint_states", self.haptx_callback, 10
        )

        # Publish joint states for fingers in Gazebo
        self.gazebo_joint_pub = self.create_publisher(
            JointState, "/left_hand/joint_states", 10
        )

        # Variables to hold latest joint states
        self.gazebo_joint_states = JointState()
        self.gazebo_joint_states.name = [
            "palm_thumb_dumb",
            "thumb_dumb_thumb_1",
            "thumb_1_thumb_2",
            "palm_index_dumb",
            "index_dumb_index_1",
            "index_1_index_2",
            "index_2_index_3",
            "palm_middle_dumb",
            "middle_dumb_middle_1",
            "middle_1_middle_2",
            "middle_2_middle_3",
            "palm_ring_dumb",
            "ring_dumb_ring_1",
            "ring_1_ring_2",
            "ring_2_ring_3",
            "palm_pinky_dumb",
            "pinky_dumb_pinky_1",
            "pinky_1_pinky_2",
            "pinky_2_pinky_3",
        ]

    def haptx_callback(self, msg):
        """
        Call whenever /haptx/lh/raw_joint_states topic gets published to.

        Args:
        ----
            msg (JointState): The joint states published by the
                hatpx driver

        """
        fa = msg.position  # finger angles

        # self.gazebo_joint_states.position = [fa['lh_THJ2'], fa['lh_THJ1'],
        # fa['lh_FFJ3'], fa['lh_FFJ2'], fa['lh_FFJ1'], fa['lh_MFJ3'],
        # fa['lh_MFJ2'], fa['lh_MFJ1'], fa['lh_RFJ3'], fa['lh_RFJ2'],
        # fa['lh_RFJ1'], fa['lh_LFJ3'], fa['lh_LFJ2'], fa['lh_LFJ1']]
        self.gazebo_joint_states.position = [
            fa[3],
            fa[4],
            fa[5],
            fa[6],
            fa[7],
            fa[8],
            fa[9],
            fa[10],
            fa[11],
            fa[12],
            fa[13],
            fa[0],
            fa[14],
            fa[15],
            fa[16],
            fa[17],
            fa[18],
            fa[19],
            fa[20],
        ]
        self.gazebo_joint_states.header.stamp = self.get_clock().now().to_msg()
        self.gazebo_joint_pub.publish(self.gazebo_joint_states)




def main(args=None):
    """fingr_joint_mapper's main function."""
    rclpy.init(args=args)

    mapper_node = Mapper()

    rclpy.spin(mapper_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mapper_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
