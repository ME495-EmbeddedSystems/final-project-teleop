import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench
from teleop_interfaces.msg import FingerWrenches
from teleop_interfaces.srv import SetWrench
from franka_driver.msg import Config

import numpy as np
import modern_robotics as mr

from sensor_msgs.msg import JointState

import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rcl_interfaces.msg import ParameterDescriptor

class HandPositionNode(Node):

    def __init__(self):
        super().__init__("hand_positioner")

        self.EE_transform = np.eye(4)

        self.declare_parameter("z_offset", 0.0,
                               ParameterDescriptor(description="The z offset for the end effector"))
        self.z = self.get_parameter("z_offset").get_parameter_value().double_value

        self.declare_parameter("yaw_offset", -3.1415926,
                               ParameterDescriptor(description="The yaw offset for the end effector"))
        self.yaw = self.get_parameter("yaw_offset").get_parameter_value().double_value

        c = np.cos(self.yaw)
        s = np.sin(self.yaw)

        self.hand_offset = np.array([[c, -s, 0, 0],
                                     [s,  c, 0, 0],
                                     [0,  0, 1, self.z],
                                     [0, 0, 0, 1]])

        self.config_sub = self.create_subscription(Config, "config", self.config_callback, 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.tmr = self.create_timer(0.01, self.timer_callback)

        self.joint_names = ['palm_x', 'palm_y', 'palm_z', 'palm_roll', 'palm_pitch', 'palm_yaw']


    def timer_callback(self):
        joints = self.convert_transform()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joints
        self.joint_pub.publish(msg)


    def config_callback(self, msg):
        arr = np.transpose(np.array([msg.config]).reshape([4, 4]))
        self.EE_transform = arr
    
    def convert_transform(self):
        T = np.matmul(self.EE_transform, self.hand_offset)
        R, p = mr.TransToRp(T)

        a = np.arctan2(-R[1][2], R[2][2])
        b = np.arcsin(R[0][2])
        g = np.arctan2(-R[0][1], R[0][0])

        joints = [p[0]-0.5, p[1]+0.0, p[2]+0.1, a, b, g]
        return joints
    
        

def main(args=None):
    """Run kinesthetic feedback node."""
    rclpy.init(args=args)
    myHPN = HandPositionNode()
    rclpy.spin(myHPN)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    