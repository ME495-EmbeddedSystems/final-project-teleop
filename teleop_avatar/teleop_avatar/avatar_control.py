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
import tf2_ros
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import numpy as np
from tf_transformations import quaternion_from_matrix, quaternion_matrix
from std_srvs.srv import Empty
import time


class AvatarControl(Node):
    """Actuate the ABB Gofas and Shadow Hands."""

    def __init__(self):
        super().__init__('avatar_control')

        # Create timer
        timer_freq = 50  # Hz
        self.timer = self.create_timer(1/timer_freq, self.timer_callback)

        # Create Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Shadow Hand publisher
        self.shadow_pub = self.create_publisher(JointTrajectory, 'rh_trajectory_controller/command', 10)
        self.joint_names = ['rh_FFJ4', 'rh_FFJ3','rh_FFJ2', 'rh_FFJ1','rh_MFJ4','rh_MFJ3','rh_MFJ2','rh_MFJ1','rh_RFJ4','rh_RFJ3','rh_RFJ2','rh_RFJ1','rh_LFJ5', 'rh_LFJ4','rh_LFJ3','rh_LFJ2','rh_LFJ1','rh_THJ5','rh_THJ4','rh_THJ3','rh_THJ2','rh_THJ1']

        # ABB Gofa Pose publisher
        self.abb_pub = self.create_publisher(TransformStamped, '/avatar/right_arm/target_tf', 10)

        # Create the /grasp service
        self.grasp = self.create_service(Grasp, "grasp", self.grasp_callback)

        # Create the /execute_trajectory service
        self.execute = self.create_service(ExecuteTrajectory, "execute_trajectory", self.execute_callback)

        # Create the /check_arm service
        self.check_arm = self.create_service(Empty, "check_arm", self.check_arm_callback)

        # Subscribe to poses
        self.obj_state_subscription = self.create_subscription(ObjectState, 'object_state', self.obj_state_callback, 10)

        # Transform of ring in hand frame
        self.T_hand_obj = np.array([[-1.0,    0.0,  0.0,       0.0],
                                    [ 0.0, 0.4695,  0.8829, -0.075],
                                    [ 0.0, 0.8829, -0.4695,   0.12],
                                    [ 0.0,    0.0,     0.0,    1.0]])

        # Transform from avatar workspace to abb table april tag
        self.T_aws_world = [[1, 0, 0,    0.4],
                            [0, 1, 0, -0.585],
                            [0, 0, 1,   0.01],
                            [0, 0, 0,      1]]
        
        # Transform of ring of interest in world frame
        self.T_world_obj = np.eye(4)
        
        self.graspStandoff = 0.1

        self.object_frame_map = {'red_ring': 'tag16H05_2'}

        self.buffer = {}

    def timer_callback(self):
        """Timer function for the Avatar Control node."""
        pass

    def grasp_callback(self, request, response):

        object_frame = self.object_frame_map[request.object_id]

        # Get necessary transforms
        world_obj_tf = self.tf_buffer.lookup_transform('tag16H05_3', object_frame, rclpy.time.Time())
        self.T_world_obj = self.transform_to_SE3(world_obj_tf)

        # Calculate transformation of hand relative to the avatar_ws
        T_aws_hand = self.T_aws_world @ self.T_world_obj @ np.linalg.inv(self.T_hand_obj)
        abb_msg = self.SE3_to_transform_stamped(T_aws_hand)
        
        # Move to standoff height above object
        abb_msg.transform.translation.z += self.graspStandoff
        
        self.abb_pub.publish(abb_msg)

        # Open hand
        point = JointTrajectoryPoint()
        point.positions = [0.027, 0.091, 0.0, 0.0, 0.147, 0.098, 0.0, 0.0, -0.216, 0.152, 0.0, 0.0, 0.0, -0.349, 0.024, 0.248, 0.0, 0.1, 1.2, 0.05, 0.15, -0.025]
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start.nanosec = 100000000 #50000000

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [point]

        self.shadow_pub.publish(msg)

        time.sleep(5)

        # Move down to object
        abb_msg.transform.translation.z -= self.graspStandoff
        
        self.abb_pub.publish(abb_msg)

        # Close hand
        point.positions = [0.027, 0.091, 0.148, 0.88, 0.147, 0.098, 0.148, 0.88, -0.216, 0.152, 0.148, 0.88, 0.0, -0.349, 0.024, 0.148, 0.88, 0.3, 1.2, 0.05, 0.15, -0.025]
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start.nanosec = 100000000 #50000000

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [point]

        self.shadow_pub.publish(msg)

        return response

    def execute_callback(self, request, response):

        # Get necessary transforms
        world_peg_tf = self.tf_buffer.lookup_transform('tag16H05_3', 'tag16H05_10', rclpy.time.Time())
        T_world_peg = self.transform_to_SE3(world_peg_tf)


        while len(self.buffer[request.object_id]) > 0:
            T_peg_obj = self.transform_to_SE3(self.buffer[request.object_id][0])

            # Calculate transformation of hand relative to avatar_ws
            T_aws_hand = np.linalg.inv(self.T_world_aws) @ T_world_peg @ T_peg_obj @ np.linalg.inv(self.T_hand_obj)
        
            abb_msg = self.SE3_to_transform_stamped(T_aws_hand)
            self.abb_pub.publish(abb_msg)

            del self.buffer[request.object_id][0]

        response.success = True

        return response
    
    def obj_state_callback(self, msg):
        # Loop through objects
        for p in msg.pose:
            object_id = p.header.frame_id

            if object_id in self.buffer:
                self.buffer[object_id] += [p.pose]
            else:
                self.buffer[object_id] = [p.pose]
    
    def transform_to_SE3(self, transform):
        """
        Converts a geometry_msgs/Transform message to an SE(3) transformation matrix.

        Args:
            transform (geometry_msgs/Transform): A transform object

        Returns:
            A numpy array representing the transform as a 4x4 SE(3) matrix

        """ 

        T = np.eye(4)
        T[0:3,0:3] = quaternion_matrix(transform.transform.rotation)
        T[0,3] = transform.transform.translation.x
        T[1,3] = transform.transform.translation.y
        T[2,3] = transform.transform.translation.z

        return T
    
    def SE3_to_transform_stamped(self, T):
        """
        Converts an SE(3) transformation matrix to a geometry_msgs/TransformStamped message.

        Args:
            T (numpy array): A 4x4 SE(3) transformation matrix

        Returns:
            A geometry_msgs/TransformStamped message to move the right ABB arm

        """ 

        # Convert transformation matrix to stamped pose data
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "operator_task_ws"
        tf.child_frame_id = "haptxRight_sr_aligned" # Right arm
        tf.transform.translation.x = T[0,3]
        tf.transform.translation.y = T[1,3]
        tf.transform.translation.z = T[2,3]
        tf.transform.rotation = quaternion_from_matrix(T[0:3, 0:3])

        return tf

def main(args=None):
    rclpy.init(args=args)

    avatar_control_node = AvatarControl()

    rclpy.spin(avatar_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avatar_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
