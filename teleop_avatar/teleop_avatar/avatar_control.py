"""
Set up arena and brick markers and moves brick.

Services:
  + grasp (teleop_interfaces/srv/Grasp) - Grasp a specific object
  + execute_trajectory (std_srvs/srv/ExecuteTrajectory) - Move the grasped object along the specified trajectory

"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from teleop_interfaces.srv import Grasp, ExecuteTrajectory
from teleop_interfaces.msg import ObjectState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from tf_transformations import quaternion_from_matrix, quaternion_matrix
from std_srvs.srv import Empty

class AvatarControl(Node):
    """Actuate the ABB Gofas and Shadow Hands."""

    def __init__(self):
        super().__init__('avatar_control')

        # Create timer
        timer_freq = 50  # Hz
        self.timer = self.create_timer(1/timer_freq, self.timer_callback)

        # Shadow Hand publisher
        self.shadow_pub = self.create_publisher(JointTrajectory, 'rh_trajectory_controller/command', 10)
        self.joint_names = ['rh_FFJ4', 'rh_FFJ3','rh_FFJ2', 'rh_FFJ1','rh_MFJ4','rh_MFJ3','rh_MFJ2','rh_MFJ1','rh_RFJ4','rh_RFJ3','rh_RFJ2','rh_RFJ1','rh_LFJ4','rh_LFJ3','rh_LFJ2','rh_LFJ1','rh_THJ5','rh_THJ4','rh_THJ3','rh_THJ2','rh_THJ1']

        # ABB Gofa Pose publisher
        self.abb_pub = self.create_publisher(PoseStamped, '/avatar/right_arm/target_tf', 10)

        # Create the /grasp service
        self.grasp = self.create_service(Grasp, "grasp", self.grasp_callback)

        # Create the /execute_trajectory service
        self.execute = self.create_service(ExecuteTrajectory, "execute_trajectory", self.execute_callback)

        # Create the /check_arm service
        self.check_arm = self.create_service(Empty, "check_arm", self.check_arm_callback)

        # Subscribe to poses
        self.obj_state_subscription = self.create_subscription(ObjectState, 'object_state', self.obj_state_callback, 10)

        self.buffer = {}

    def timer_callback(self):
        """Timer function for the Avatar Control node."""
        pass

    def grasp_callback(self, request, response):
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.24, 0.0, 0.66, 0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 1.1, 0.0, 0.0, 0.0, 0.93, -0.17, 1.22, -0.03, 0.26, 0.03]
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start.nanosec = 100000000 #50000000

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [point]

        self.shadow_pub.publish(msg)

        point.positions = [0.0, 0.24, 0.0, 0.75, 0.0, 0.0, 0.0, 1.3, 0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 1.0, -0.17, 1.22, -0.03, 0.26, 0.03]
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start.nanosec = 100000000 #50000000

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [point]

        self.shadow_pub.publish(msg)

        return response

    def execute_callback(self, request, response):
        while len(self.buffer) > 0:
            # go_to(self.buffer[request.object_id][0])

            del self.buffer[0]

        response.success = True

        return response
    
    def obj_state_callback(self, msg):
        # Loop through objects
        for i in range(len(msg.stamp)):
            object_id = msg.stamp[i].frame_id

            if object_id in self.buffer:
                self.buffer[object_id] += [msg.pose[i]]
            else:
                self.buffer[object_id] = [msg.pose[i]]

    def grasp_pose(self, pose_w_obj):
        """Takes in pose of object in world frame and outputs pose of hand in world frame to grasp the object"""

        # Load stamped pose data as transformation matrix
        T_w_obj = np.eye(4)
        T_w_obj[0:3,0:3] = quaternion_matrix(pose_w_obj.pose.orientation)
        T_w_obj[0,3] = pose_w_obj.pose.position.x
        T_w_obj[1,3] = pose_w_obj.pose.position.y
        T_w_obj[2,3] = pose_w_obj.pose.position.z

        # State transformation of object relative to hand
        T_hand_obj = np.eye(4)
        T_hand_obj[0:3, 0:3] = [[0,1,0],[0,0,1],[1,0,0]] # Rotation to make z of object align with y of hand
        T_hand_obj[0,3] = 0.00 # x-offset, distance above thumb for right hand and below pinky for left hand
        T_hand_obj[1,3] = -0.05 # y-offset, distance above the back of the wrist
        T_hand_obj[2,3] = 0.09 # z-offset, distance in the direction of the fingers
             
        # Calculate transformation of hand relative to world
        T_w_hand = T_w_obj @ T_hand_obj.inv

        # Convert transformation matrix to stamped pose data
        pose_w_hand = PoseStamped()
        pose_w_hand.header.stamp = pose_w_obj.header.stamp
        pose_w_hand.pose.position.x = T_w_hand[0,3]
        pose_w_hand.pose.position.y = T_w_hand[1,3]
        pose_w_hand.pose.position.z = T_w_hand[2,3]
        pose_w_hand.pose.orientation = quaternion_from_matrix(T_w_hand[0:3, 0:3])

        return pose_w_hand

    def check_arm_callback(self, request, response):

        # Command arm to first pose

        pose_w_obj = PoseStamped()
        pose_w_obj.header.stamp = self.get_clock().now().to_msg()
        pose_w_obj.pose.position.x = 0.5
        pose_w_obj.pose.position.y = 0.5
        pose_w_obj.pose.position.z = 0.5
        pose_w_obj.pose.orientation = quaternion_from_matrix(np.eye(3))
        
        pose_w_hand = self.grasp_pose(pose_w_obj)

        self.abb_pub.publish(pose_w_hand)

        # Command arm to second pose

        pose_w_obj.header.stamp = self.get_clock().now().to_msg()
        pose_w_obj.pose.position.x = 0.5
        pose_w_obj.pose.position.y = -0.5
        pose_w_obj.pose.position.z = 0.5
        pose_w_obj.pose.orientation = quaternion_from_matrix(np.eye(3))
        
        pose_w_hand = self.grasp_pose(pose_w_obj)

        self.abb_pub.publish(pose_w_hand)

        return response

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
