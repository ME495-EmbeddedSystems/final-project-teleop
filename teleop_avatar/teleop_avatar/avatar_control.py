"""
Set up arena and brick markers and moves brick.

Services:
  + grasp (teleop_interfaces/srv/Grasp) - Grasp a specific object
  + execute_trajectory (std_srvs/srv/ExecuteTrajectory) - Move the grasped object along the specified trajectory

"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from teleop_interfaces.srv import Grasp, ExecuteTrajectory
from teleop_interfaces.msg import ObjectState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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

        # Create the /grasp service
        self.grasp = self.create_service(Grasp, "grasp", self.grasp_callback)

        # Create the /execute_trajectory service
        self.execute = self.create_service(ExecuteTrajectory, "execute_trajectory", self.execute_callback)

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
