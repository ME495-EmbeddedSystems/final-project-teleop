"""
Actuate HaptX Gloves.

Subscribers:
  + haptx/rh/tactor_group_state (haptx_interfaces/msg/TactorGroupState) - Actuate tactor groups on right glove
  + haptx/lh/tactor_group_state (haptx_interfaces/msg/TactorGroupState) - Actuate tactor groups on left glove

Parameters
----------
  + use_right (bool) - Determines whether right glove is being used
  + use_left (bool) - Determines whether left glove is being used

"""
import rclpy
from rclpy.node import Node
from array import array
from geometry_msgs.msg import WrenchStamped
from haptx_interfaces.msg import TactorGroupState, FingerBrakeState
from std_srvs.srv import SetBool


class HaptxFeedback(Node):
    """Node to actuate the HaptX Gloves."""

    def __init__(self):
        super().__init__('haptx_feedback')

        # Declare and get parameters
        self.declare_parameter('use_right', True)
        self.declare_parameter('use_left', True)
        self.use_right = self.get_parameter('use_right').get_parameter_value().bool_value
        self.use_left = self.get_parameter('use_left').get_parameter_value().bool_value

        # Create control loop timer based on frequency parameter
        timer_freq = 50  # Hz
        self.timer = self.create_timer(1/timer_freq, self.timer_callback)

        # Create publishers for tactor groups
        if self.use_right:
            self.rh_tactor_pub = self.create_publisher(TactorGroupState, "haptx/rh/tactor_group_state", 10)
            self.rh_brake_pub = self.create_publisher(FingerBrakeState, "haptx/rh/brake_state", 10)
        
        if self.use_left:
            self.lh_tactor_pub = self.create_publisher(TactorGroupState, "haptx/lh/tactor_group_state", 10)
            self.lh_brake_pub = self.create_publisher(FingerBrakeState, "haptx/lh/brake_state", 10)

        self.right_tactors_on = False
        self.left_tactors_on = False

        # Create grasped service
        self.grasped = self.create_service(SetBool, 'grasped', self.grasped_callback)

    def timer_callback(self):
        if self.use_right:
            tactor_msg = TactorGroupState()
            tactor_msg.tactor_group = ['rh_th', 'rh_ff', 'rh_mf', 'rh_rf', 'rh_lf']
            
            if self.right_tactors_on == True:
                tactor_msg.inflation = [1.0]*5
            else:
                tactor_msg.inflation = [0.0]*5

            self.rh_tactor_pub.publish(tactor_msg)

        if self.use_left:
            tactor_msg = TactorGroupState()
            tactor_msg.tactor_group = ['lh_th', 'lh_ff', 'lh_mf', 'lh_rf', 'lh_lf']
            
            if self.left_tactors_on == True:
                tactor_msg.inflation = [1.0]*5
            else:
                tactor_msg.inflation = [0.0]*5
                    
            self.lh_tactor_pub.publish(tactor_msg)

    def grasped_callback(self, request, response):
        """Timer function for the Haptx Feedback node."""

        if self.use_right:
            brake_msg = FingerBrakeState()
            brake_msg.name = ['rh_th', 'rh_ff', 'rh_mf', 'rh_rf', 'rh_lf']
            
            if request.data == True:
                self.right_tactors_on = True
                brake_msg.activated = [True]*5
            else:
                self.right_tactors_on = False
                brake_msg.activated = [False]*5

            self.rh_brake_pub.publish(brake_msg)

        if self.use_left:
            brake_msg = FingerBrakeState()
            brake_msg.name = ['lh_th', 'lh_ff', 'lh_mf', 'lh_rf', 'lh_lf']
            
            if request.data == True:
                self.right_tactors_on = True
                brake_msg.activated = [True]*5
            else:
                self.right_tactors_on = False
                brake_msg.activated = [False]*5

            self.lh_brake_pub.publish(brake_msg)

        response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)

    haptx_feedback_node = HaptxFeedback()

    rclpy.spin(haptx_feedback_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    haptx_feedback_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
