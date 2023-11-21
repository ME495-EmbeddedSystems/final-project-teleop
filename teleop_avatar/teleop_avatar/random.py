"""
Set up arena and brick markers and moves brick.

Services:
  + grasp (teleop_interfaces/srv/Grasp) - Grasp a specific object
  + execute_trajectory (std_srvs/srv/ExecuteTrajectory) - Move the grasped object along the specified trajectory

"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from teleop_interfaces.srv import Grasp, ExecuteTrajectory
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Quaternion
from teleop_interfaces.srv import Grasp, ExecuteTrajectory
from teleop_interfaces.msg import ObjectState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import numpy as np
from tf_transformations import quaternion_from_matrix, quaternion_matrix
from std_srvs.srv import Empty
import time

def SE3toQuaternion(T):
    """
    Converts SE(3) matrix to quaternion.

    Args:
        T (4*4 array like) : SE(3) matrix, even for pure rotations.

    Returns:
        quat (geometry_msgs/msg/Quaternion) : Quaternion correspaonding to rotation portion of transformation.

    """ 

    quaternion_array = quaternion_from_matrix(T)

    quat = Quaternion()
    quat.x = quaternion_array[0]
    quat.y = quaternion_array[1]
    quat.z = quaternion_array[2]
    quat.w = quaternion_array[3]

    return quat

def QuaterniontoSE3(quat):
    """
    Converts quaternion message to SE(3).

    Args:
        quat (geometry_msgs/msg/Quaternion) : Quaternion correspaonding to rotation portion of transformation.

    Returns:
        T (4*4 array like) : SE(3) matrix, even for pure rotations.

    """ 

    quaternion_array = [quat.x, quat.y, quat.z, quat.w]
    T = np.array(quaternion_matrix(quaternion_array))

    return T


class Random(Node):
    """Actuate the ABB Gofas and Shadow Hands."""

    def __init__(self):
        super().__init__('random')

        # Create timer
        timer_freq = 50  # Hz
        self.timer = self.create_timer(1/timer_freq, self.timer_callback)

        # Create Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create the /ping service
        self.ping = self.create_service(Empty, "ping", self.callback_func)

        # Shadow Hand publisher
        self.shadow_wr_pub = self.create_publisher(JointTrajectory, 'rh_wr_trajectory_controller/command', 10)
        self.wr_joint_names = ['rh_WRJ1', 'rh_WRJ2']

        # Shadow Hand publisher
        self.shadow_pub = self.create_publisher(JointTrajectory, 'rh_trajectory_controller/command', 10)
        self.joint_names = ['rh_FFJ4', 'rh_FFJ3','rh_FFJ2', 'rh_FFJ1','rh_MFJ4','rh_MFJ3','rh_MFJ2','rh_MFJ1','rh_RFJ4','rh_RFJ3','rh_RFJ2','rh_RFJ1','rh_LFJ5', 'rh_LFJ4','rh_LFJ3','rh_LFJ2','rh_LFJ1','rh_THJ5','rh_THJ4','rh_THJ3','rh_THJ2','rh_THJ1']

        # world_peg_tf = self.tf_buffer.lookup_transform('tag16H05_3', 'tag16H05_10', rclpy.time.Time())
        # T_world_peg = self.transform_to_SE3(world_peg_tf)

    def timer_callback(self):
        """Timer function for the Avatar Control node."""
        # point = JointTrajectoryPoint()
        # point.positions = [0.0, 0.0]
        # point.velocities = [0.0] * len(self.wr_joint_names)
        # point.time_from_start.nanosec = 100000000 #50000000

        # msg = JointTrajectory()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.joint_names = self.wr_joint_names
        # msg.points = [point]

        # self.shadow_wr_pub.publish(msg)
        if self.tf_buffer.can_transform('cmd/gofa2_base', 'cmd/avatar_task_ws', rclpy.time.Time()):
            abb_avatarws_tf = self.tf_buffer.lookup_transform(
                    'cmd/gofa2_base', 'cmd/avatar_task_ws', rclpy.time.Time())
            self.get_logger().info(str(abb_avatarws_tf))

    def callback_func(self, request, response):
        point = JointTrajectoryPoint()
        point.positions = [0.027, 0.091, 0.0, 0.0, 0.147, 0.098, 0.0, 0.0, -0.216, 0.152, 0.0, 0.0, 0.0, -0.349, 0.024, 0.248, 0.0, 0.3, 0.8, 0.05, 0.15, -0.025]
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start.nanosec = 100000000 #50000000

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [point]

        self.shadow_pub.publish(msg)

        return response


def main(args=None):
    rclpy.init(args=args)

    random_node = Random()

    rclpy.spin(random_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    random_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
