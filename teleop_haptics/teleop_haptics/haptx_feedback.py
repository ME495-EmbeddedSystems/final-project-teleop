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
