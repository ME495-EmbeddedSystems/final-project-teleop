import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench
# from teleop_interfaces.msg import FingerWrenches
from teleop_interfaces.srv import SetWrench

import numpy as np

import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rcl_interfaces.msg import ParameterDescriptor


class KinestheticFeedbackNode(Node):
    """
    This node takes in an array of wrench contact forces and produces joint torques provided to recreate the feedback. 

    Parameters
    ----------
        - force_max (float) : The maximum allowed applied force
        - torque_max (float) : The maximum allowed applied torque

    Publishers
    ----------
        - /applied_wrench (geometry_msgs/msg/Wrench) : The wrench the frankas will apply
    Services
    --------
        - /set_applied_wrench (teleop_interfaces/srv/SetWrench) : Sets the wrench the frankas will apply.
    """

    def __init__(self):
        super().__init__("kinesthetic_feedback")

        self.contact_wrench = Wrench()
        #self.finger_wrenches = FingerWrenches()

        self.declare_parameter("force_max", 40.0,
                               ParameterDescriptor(description="The maximum allowed applied force"))
        self.force_max = self.get_parameter("force_max").get_parameter_value().double_value

        self.declare_parameter("torque_max", 1.0,
                               ParameterDescriptor(description="The maximum allowed applied torque"))
        self.torque_max = self.get_parameter("torque_max").get_parameter_value().double_value

        # To be implemented in the future with force inputs.
        # self.contact_wrench_sub = self.create_subscription(FingerWrenches, "force_torque", self.contact_wrench_callback, 10)
        self.applied_wrench_set = self.create_service(SetWrench, "set_applied_wrench", self.applied_wrench_set_callback)
        self.applied_wrench_pub = self.create_publisher(Wrench, "applied_wrench", 10)

        self.tmr = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        """Publish the applied wrench."""
        self.applied_wrench_pub.publish(self.contact_wrench) 
    
    def applied_wrench_set_callback(self, req, response):
        """Set the applied wrench."""
        self.contact_wrench = req.wrench
        self.limit_wrench()
        return response
    
    def limit_wrench(self):
        """Saftey limit the wrench."""
        force_mag = np.sqrt(self.contact_wrench.force.x**2 + self.contact_wrench.force.y**2 + self.contact_wrench.force.z**2)
        if(force_mag > self.force_max):
            self.contact_wrench.force.x * force_mag / self.force_max
            self.contact_wrench.force.y * force_mag / self.force_max
            self.contact_wrench.force.z * force_mag / self.force_max
        
        torque_mag = np.sqrt(self.contact_wrench.torque.x**2 + self.contact_wrench.torque.y**2 + self.contact_wrench.torque.z**2)
        if(torque_mag > self.torque_max):
            self.contact_wrench.torque.x * torque_mag / self.torque_max
            self.contact_wrench.torque.y * torque_mag / self.torque_max
            self.contact_wrench.torque.z * torque_mag / self.torque_max


def main(args=None):
    """Run kinesthetic feedback node."""
    rclpy.init(args=args)
    myKFN = KinestheticFeedbackNode()
    rclpy.spin(myKFN)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    