"""
Actuate HaptX Gloves.

Subscribers:
  + haptx/rh/tactor_group_state (haptx_interfaces/msg/TactorGroupState) - Actuate tactor groups on right glove
  + haptx/lh/tactor_group_state (haptx_interfaces/msg/TactorGroupState) - Actuate tactor groups on left glove
  + haptx/rh/brake_state (haptx_interfaces/msg/FingerBrakeState) - Actuate tactor groups on right glove
  + haptx/lh/brake_state (haptx_interfaces/msg/FingerBrakeState) - Actuate tactor groups on left glove

Services:
  + grasped (std_srvs/srv/SetBool) - Turn HaptX Glove's "grasp sensation" on/off

Parameters
----------
  + side (string) - Determines whether to use the left glove, right glove, or both

"""
import rclpy
from rclpy.node import Node
from haptx_interfaces.msg import TactorGroupState, FingerBrakeState
from std_srvs.srv import SetBool


class HaptxFeedback(Node):
    """Node to actuate the HaptX Gloves."""

    def __init__(self):
        super().__init__('haptx_feedback')

        # Declare and get parameters
        self.declare_parameter('side', 'both')
        self.side = self.get_parameter('side').get_parameter_value().string_value

        # Create control loop timer based on frequency parameter
        timer_freq = 100  # Hz
        self.timer = self.create_timer(1/timer_freq, self.timer_callback)

        # Create publishers for tactor groups and finger brakes
        if self.side == 'right' or self.side == 'both':
            self.rh_tactor_pub = self.create_publisher(TactorGroupState, "haptx/rh/tactor_group_state", 10)
            self.rh_brake_pub = self.create_publisher(FingerBrakeState, "haptx/rh/brake_state", 10)
        
        if self.side == 'left' or self.side == 'both':
            self.lh_tactor_pub = self.create_publisher(TactorGroupState, "haptx/lh/tactor_group_state", 10)
            self.lh_brake_pub = self.create_publisher(FingerBrakeState, "haptx/lh/brake_state", 10)

        # Create grasped service
        self.grasped = self.create_service(SetBool, 'grasped', self.grasped_callback)

        # Variables to store current state of HaptX Gloves
        self.right_tactors_on = False
        self.left_tactors_on = False

    def timer_callback(self):
        """Timer callback for HaptX Feedback node."""

        # If right glove is being used
        if self.side == 'right' or self.side == 'both':
            tactor_msg = TactorGroupState()
            tactor_msg.tactor_group = ['rh_th', 'rh_ff', 'rh_mf', 'rh_rf', 'rh_lf']
            
            # If right tactors should be on, publish inflation values of 1.0 for all fingers
            if self.right_tactors_on == True:
                tactor_msg.inflation = [1.0]*5
            # Otherwise, publish inflation values of 0.0 for all fingers
            else:
                tactor_msg.inflation = [0.0]*5

            self.rh_tactor_pub.publish(tactor_msg)

        # If left glove is being used
        if self.side == 'left' or self.side == 'both':
            tactor_msg = TactorGroupState()
            tactor_msg.tactor_group = ['lh_th', 'lh_ff', 'lh_mf', 'lh_rf', 'lh_lf']
            
            # If left tactors should be on, publish inflation values of 1.0 for all fingers
            if self.left_tactors_on == True:
                tactor_msg.inflation = [1.0]*5
            # Otherwise, publish inflation values of 0.0 for all fingers
            else:
                tactor_msg.inflation = [0.0]*5
                    
            self.lh_tactor_pub.publish(tactor_msg)

    def grasped_callback(self, request, response):
        """
        Turns the "grasping sensation" of the HaptX Gloves on and off.

        Args
        ----
            request (SetBoolRequest): A SetBool request object

            response (SetBool): The response object

        Returns
        -------
           A SetBool response

        """

        # If right glove is being used
        if self.side == 'right' or self.side == 'both':
            brake_msg = FingerBrakeState()
            brake_msg.name = ['rh_th', 'rh_ff', 'rh_mf', 'rh_rf', 'rh_lf']
            
            # If request contains 'True' boolean, turn right tactors on and activate finger brakes
            if request.data == True:
                self.right_tactors_on = True
                brake_msg.activated = [True]*5
            # Otherwise, turn right tactors off and deactivate finger brakes
            else:
                self.right_tactors_on = False
                brake_msg.activated = [False]*5

            self.rh_brake_pub.publish(brake_msg)

         # If right glove is being used
        if self.side == 'left' or self.side == 'both':
            brake_msg = FingerBrakeState()
            brake_msg.name = ['lh_th', 'lh_ff', 'lh_mf', 'lh_rf', 'lh_lf']
            
            # If request contains 'True' boolean, turn left tactors on and activate finger brakes
            if request.data == True:
                self.left_tactors_on = True
                brake_msg.activated = [True]*5
            # Otherwise, turn left tactors off and deactivate finger brakes
            else:
                self.left_tactors_on = False
                brake_msg.activated = [False]*5

            self.lh_brake_pub.publish(brake_msg)

        # Set success to 'True' in response
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
