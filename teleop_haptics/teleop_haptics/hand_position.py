import rclpy
from rclpy.node import Node

from franka_driver.msg import Config

import numpy as np
import modern_robotics as mr

from sensor_msgs.msg import JointState

from rcl_interfaces.msg import ParameterDescriptor

class HandPositionNode(Node):
    """
    This node provides the hand postion based on the franka robot configuration.

    Parameters:
    ----------
        - z_offset (float) : The z offset of the hand from the end effector
        - yaw_offset (float) : The yaw offset of the hand from the end effector
        - x_world_offset (float) : The x origin offset of the coordinate system used.
        - y_world_offset (float) : The y origin offset of the coordinate system used.
        - z_world_offset (float) : The z origin offset of the coordinate system used.
    
    Subscriptions:
    -------------
        - /config (franka_driver/msg/Config) : The configuration of the end effector

    Publishers:
    ----------
        - /joint_states (sensor_msgs/msg/JointState) : The joint states of the palm (xyz, rpy)

    """

    def __init__(self):
        super().__init__("hand_positioner")

        self.EE_transform = np.eye(4)

        self.declare_parameter("z_offset", 0.0,
                               ParameterDescriptor(description="The z offset for the end effector"))
        self.z = self.get_parameter("z_offset").get_parameter_value().double_value

        self.declare_parameter("yaw_offset", -np.pi/2,
                               ParameterDescriptor(description="The yaw offset for the end effector"))
        self.yaw = self.get_parameter("yaw_offset").get_parameter_value().double_value


        self.declare_parameter("x_world_offset", 0.6,
                               ParameterDescriptor(description="The x world offset for the coordinate frame"))
        self.x_world = self.get_parameter("x_world_offset").get_parameter_value().double_value

        self.declare_parameter("y_world_offset", 0.0,
                               ParameterDescriptor(description="The y world offset for the coordinate frame"))
        self.y_world = self.get_parameter("y_world_offset").get_parameter_value().double_value

        self.declare_parameter("z_world_offset", 0.0,
                               ParameterDescriptor(description="The z world offset for the coordinate frame"))
        self.z_world = self.get_parameter("z_world_offset").get_parameter_value().double_value

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
        """Publish joint states."""
        joints = self.convert_transform()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joints
        self.joint_pub.publish(msg)

    def config_callback(self, msg):
        """Pull configuration from franka controller."""
        arr = np.transpose(np.array([msg.config]).reshape([4, 4]))
        self.EE_transform = arr
    
    def convert_transform(self):
        """Convert to T_world_hand."""
        T = np.matmul(self.EE_transform, self.hand_offset)
        R, p = mr.TransToRp(T)

        a = np.arctan2(-R[1][2], R[2][2]) # Roll
        b = np.arcsin(R[0][2]) # Pitch
        g = np.arctan2(-R[0][1], R[0][0]) # Yaw

        joints = [p[0]-self.x_world, p[1]-self.y_world, p[2]-self.z_world, a, b, g]
        return joints
    
        

def main(args=None):
    """Run hand positioner node."""
    rclpy.init(args=args)
    myHPN = HandPositionNode()
    rclpy.spin(myHPN)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    