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
from haptx_interfaces.msg import TactorGroupState


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
        
        if self.use_left:
            self.lh_tactor_pub = self.create_publisher(TactorGroupState, "haptx/lh/tactor_group_state", 10)

        self.inflation = 0.0
        self.dir = 'up'

        # Center tactors
        self.rh_th_center_tactors = [0061, 0071, 0081, 0031, 0041, 0101, 0091]
        self.rh_ff_center_tactors = [1061, 1071, 1081, 1031, 1041, 1101, 1091]
        self.rh_mf_center_tactors = [2061, 2071, 2081, 2031, 2041, 2101, 2091]
        self.rh_rf_center_tactors = [3032, 3031, 3022, 3052]
        self.rh_lf_center_tactors = [4032, 4031, 4022, 4052]

        self.lh_th_center_tactors = [10061, 10071, 10081, 10031, 10041, 10101, 10091]
        self.lh_ff_center_tactors = [11061, 11071, 11081, 11031, 11041, 11101, 11091]
        self.lh_mf_center_tactors = [12061, 12071, 12081, 12031, 12041, 12101, 12091]
        self.lh_rf_center_tactors = [13032, 13031, 13022, 13052]
        self.lh_lf_center_tactors = [14032, 14031, 14022, 14052]

    def timer_callback(self):
        """Timer function for the Haptx Feedback node."""
        msg = TactorGroupState()
        msg.tactor_group = ['lh_th', 'lh_mf', 'lh_rf']
        msg.inflation = [self.inflation, self.inflation, self.inflation]

        self.lh_tactor_pub.publish(msg)

        if self.dir == 'up':
            if self.inflation < 1.0:
                self.inflation += 0.05
            else:
                self.dir = 'down'
        elif self.dir == 'down':
            if self.inflation > 0.0:
                self.inflation -= 0.05
            else:
                self.dir = 'up'


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
