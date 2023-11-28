import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench
from teleop_interfaces.msg import FingerWrenches
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
    """

    def __init__(self):
        super().__init__("kinesthetic_feedback")

        self.contact_wrench = Wrench()
        self.finger_wrenches = FingerWrenches()

        self.contact_wrench_sub = self.create_subscription(FingerWrenches, "force_torque", self.contact_wrench_callback, 10)
        self.applied_wrench_set = self.create_service(SetWrench, "set_applied_wrench", self.applied_wrench_set_callback)
        self.applied_wrench_pub = self.create_publisher(Wrench, "applied_wrench", 10)

        self.tmr = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        self.applied_wrench_pub.publish(self.contact_wrench)

    def contact_wrench_callback(self, msg):
        self.finger_wrenches = msg
        self.limit_wrench()
    
    #def convertAndSumWrenches(self):
    #    for i in self.finger_wrenches.finger_wrenches:
    #        
    
    def applied_wrench_set_callback(self, req, response):
        self.contact_wrench = req.wrench
        self.limit_wrench()
        return response
    
    def limit_wrench(self):
        force_mag = np.sqrt(self.contact_wrench.force.x**2 + self.contact_wrench.force.y**2 + self.contact_wrench.force.z**2)
        if(force_mag > 40):
            self.contact_wrench.force.x *force_mag / 40
            self.contact_wrench.force.y *force_mag / 40
            self.contact_wrench.force.z *force_mag / 40
        
        torque_mag = np.sqrt(self.contact_wrench.torque.x**2 + self.contact_wrench.torque.y**2 + self.contact_wrench.torque.z**2)
        if(torque_mag > 5):
            self.contact_wrench.torque.x *torque_mag / 5
            self.contact_wrench.torque.y *torque_mag / 5
            self.contact_wrench.torque.z *torque_mag / 5


def main(args=None):
    """Run kinesthetic feedback node."""
    rclpy.init(args=args)
    myKFN = KinestheticFeedbackNode()
    rclpy.spin(myKFN)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    