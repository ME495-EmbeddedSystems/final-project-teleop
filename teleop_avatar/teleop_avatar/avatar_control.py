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

def SE3toQuaternion(T):
    """
    Converts SE(3) matrix to quaternion.

    Args:
        T (4x4 array like) : SE(3) matrix, even for pure rotations.

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
        T (4x4 array like) : SE(3) matrix, even for pure rotations.

    """ 

    quaternion_array = [quat.x, quat.y, quat.z, quat.w]
    T = np.array(quaternion_matrix(quaternion_array))

    return T


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

        # Create the /move_to_april service
        self.move_to_april = self.create_service(Empty, "move_to_april", self.move_to_april_callback)

        # Subscribe to poses
        self.obj_state_subscription = self.create_subscription(ObjectState, 'object_state', self.obj_state_callback, 10)

        # Transform of ring in hand frame
        self.T_hand_ring = np.eye(4)

        # Transform of abb base in world frame
        self.T_world_abb = np.eye(4)

        # Transform from avatar_task_ws to abb table april tag
        self.T_aws_abbAT = [[1, 0, 0, 0.4], [0, 1, 0, -0.585], [0, 0, 1, 0.01], [0, 0, 0, 1]]
        self.targetStandoffHeight = 0.1

        self.buffer = {}
        self.buffer['red'] = [0,1,2,3,4,7,8,9]

    def timer_callback(self):
        """Timer function for the Avatar Control node."""
        pass

    def grasp_callback(self, request, response):

        # Get necessary transforms
        world_peg_tf = self.tf_buffer.lookup_transform('tag16H05_3', 'tag16H05_10', rclpy.time.Time())
        T_world_peg = self.transform_to_SE3(world_peg_tf)

        peg_obj_tf = self.tf_buffer.lookup_transform('tag16H05_10', 'red_ring', rclpy.time.Time())
        T_peg_obj = self.transform_to_SE3(peg_obj_tf)

        # Calculate transformation of hand relative to ABB base
        T_abb_hand = np.linalg.inv(self.T_world_abb) @ T_world_peg @ T_peg_obj @ np.linalg.inv(self.T_hand_obj)

        abb_msg = self.SE3_to_transform_stamped(T_abb_hand)
        self.abb_pub.publish(abb_msg)

        point = JointTrajectoryPoint()
        point.positions = [0.027, 0.091, 0.0, 0.0, 0.147, 0.098, 0.0, 0.0, -0.216, 0.152, 0.0, 0.0, 0.0, -0.349, 0.024, 0.248, 0.0, 0.3, 0.8, 0.05, 0.15, -0.025]
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start.nanosec = 100000000 #50000000

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [point]

        self.shadow_pub.publish(msg)

        time.sleep(2)

        point.positions = [0.027, 0.091, 0.379, 0.429, 0.147, 0.098, 0.551, 0.67, -0.216, 0.152, 0.589, 0.706, 0.0, -0.349, 0.024, 0.248, 1.08, 0.46, 1.0, 0.05, 0.15, -0.025]
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start.nanosec = 100000000 #50000000

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [point]

        self.shadow_pub.publish(msg)

        return response

    def execute_callback(self, request, response):
        while len(self.buffer[request.object_id]) > 0:
            # go_to(self.buffer[request.object_id][0])

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
            

    def grasp_transform(self, object_frame_id):
        """ 
        Takes in id of transform representing object and outputs transform of palm to grasp the object.
        All transforms are in avatar_task_ws frame.

        """

        # TF Listener

        tf_detected = False # Flag for transform being recorded successfully
        timedout = False # Flag for time out condition
        startTime = time.time() 
        currentTime = startTime
        timeoutmillis = 5000 # Timeout time

        while(not tf_detected and not timedout):
        
            currentTime = time.time()
            if currentTime - startTime > timeoutmillis:
                timedout = True

            try:
                # get the latest transform from abb table april tag to object_frame_id.
                T_abbAT_obj = self.transform_to_SE3(self.tf_buffer.lookup_transform("tag16H05_3", object_frame_id, rclpy.time.Time()))
                tf_detected = True
                self.get_logger().info("TRANSFORM RECORDED")

            except tf2_ros.LookupException as e:
                # the frames don't exist yet
                self.get_logger().info(f"Frames don't exist yet: {e}")
            except tf2_ros.ConnectivityException as e:
                # the tf tree has a disconnection
                self.get_logger().info(f"Connectivity exception: {e}")
            except tf2_ros.ExtrapolationException as e:
                # the times are two far apart to extrapolate
                self.get_logger().info(f"Extrapolation exception: {e}")

        # Transform from avatar_task_ws to workspace table april tag.
        T_aws_obj = self.T_aws_abbAT @ T_abbAT_obj

        # State transformation of object relative to hand
        T_hand_obj = np.eye(4)
        T_hand_obj[0:3, 0:3] = [[-1,0,0],[0,0,1],[0,1,0]] # Rotation to make z of object align with y of hand
        # T_hand_obj[0:3,0:3] = np.array([[-1,0,0],[0,0,1],[0,1,0]]) @ np.array([[1, 0, 0], [0, 0.909, -0.417], [0, 0.417, 0.909]])
        # T_hand_obj[0:3,0:3] = np.array([[-1,0,0],[0,0,1],[0,1,0]]) @ np.array([[1, 0, 0], [0, 0.9063, -0.4226], [0, 0.4226, 0.9063]]) # 25
        T_hand_obj[0:3,0:3] = np.array([[-1,0,0],[0,0,1],[0,1,0]]) @ np.array([[1, 0, 0], [0, 0.8829, -0.4695], [0, 0.4695, 0.8829]]) # 28
        # T_hand_obj[0:3,0:3] = np.array([[-1,0,0],[0,0,1],[0,1,0]]) @ np.array([[1, 0, 0], [0, 0.866, -0.5], [0, 0.5, 0.866]]) # 30
        T_hand_obj[0,3] = 0.00 # x-offset, distance above thumb for right hand and below pinky for left hand
        T_hand_obj[1,3] = -0.075 # y-offset, distance above the back of the wrist
        T_hand_obj[2,3] = 0.12 # z-offset, distance in the direction of the fingers
             
        # Calculate transformation of hand relative to ABB base
        T_aws_hand = T_aws_obj @ np.linalg.inv(T_hand_obj)
        self.get_logger().info("Math solved")

        tf_aws_hand = self.SE3_to_transform_stamped(T_aws_hand)

        return tf_aws_hand

    def check_arm_callback(self, request, response):

        # Command arm to first pose
        pose_peg_obj = PoseStamped()
        pose_peg_obj.header.stamp = self.get_clock().now().to_msg()
        pose_peg_obj.pose.position.x = 0.5
        pose_peg_obj.pose.position.y = 0.5
        pose_peg_obj.pose.position.z = 0.5
        pose_peg_obj.pose.orientation = quaternion_from_matrix(np.eye(3))
        
        pose_peg_obj.pose.position.y = 0.1
        pose_peg_obj.pose.position.z = 0.3
        pose_peg_obj.pose.orientation = SE3toQuaternion(np.eye(4))

        tf_abb_hand = self.grasp_transform(pose_peg_obj)

        self.abb_pub.publish(tf_abb_hand)

        self.get_logger().info("Published first pose")

        time.sleep(2)

        # Command arm to second pose
        pose_peg_obj = PoseStamped()
        pose_peg_obj.header.stamp = self.get_clock().now().to_msg()
        pose_peg_obj.pose.position.x = 0.5
        pose_peg_obj.pose.position.y = -0.5
        pose_peg_obj.pose.position.z = 0.5
        pose_peg_obj.pose.orientation = quaternion_from_matrix(np.eye(3))
        pose_peg_obj.pose.position.y = -0.1
        pose_peg_obj.pose.position.z = 0.3
        pose_peg_obj.pose.orientation = np.eye(4)
        pose_peg_obj.pose.orientation = SE3toQuaternion(np.eye(4))
        
        tf_abb_hand = self.grasp_transform(pose_peg_obj)

        self.abb_pub.publish(tf_abb_hand)
        self.get_logger().info("Published second pose")

        return response
    
    def move_to_april_callback(self, request, response):
        """Moves to a standoff position above the april tag"""

        tf_aws_hand = self.grasp_transform("tag16H05_2")
        tf_aws_hand.transform.translation.z = tf_aws_hand.transform.translation.z + self.targetStandoffHeight

        # Publish standoff position
        self.abb_pub.publish(tf_aws_hand)

        self.get_logger().info("Published Hand transform - standoff")
        time.sleep(5)

        tf_aws_hand.transform.translation.z = tf_aws_hand.transform.translation.z - self.targetStandoffHeight

        # Publish standoff position
        self.abb_pub.publish(tf_aws_hand)

        self.get_logger().info("Published Hand transform - grasp")

        time.sleep(5)

        return response
    
    def transform_to_SE3(self, transform):
        """
        Converts a geometry_msgs/Transform message to an SE(3) transformation matrix.

        Args:
            transform (geometry_msgs/Transform): A transform object

        Returns:
            A numpy array representing the transform as a 4x4 SE(3) matrix

        """ 

        T = np.eye(4)
        T[:,:] = QuaterniontoSE3(transform.transform.rotation)
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
        tf.transform.rotation = SE3toQuaternion(T[:, :])

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
