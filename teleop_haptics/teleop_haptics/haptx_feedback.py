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
from haptx_interfaces.msg import TactorState


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
            self.rh_tactor_pub = self.create_publisher(TactorState, "haptx/rh/tactor_state", 10)
        
        if self.use_left:
            self.lh_tactor_pub = self.create_publisher(TactorState, "haptx/lh/tactor_state", 10)

        self.contact_wrench_sub = self.create_subscription(WrenchStamped, "force_torque", self.contact_wrench_callback, 10)

        # Center tactors
        self.rh_th_center_tactors = [61, 71, 81, 31, 41, 101, 91]
        self.rh_ff_center_tactors = [1061, 1071, 1081, 1031, 1041, 1101, 1091]
        self.rh_mf_center_tactors = [2061, 2071, 2081, 2031, 2041, 2101, 2091]
        self.rh_rf_center_tactors = [3032, 3031, 3022, 3052]
        self.rh_lf_center_tactors = [4032, 4031, 4022, 4052]
        self.rh_center_tactors = [self.rh_ff_center_tactors, self.rh_mf_center_tactors, self.rh_rf_center_tactors, self.rh_lf_center_tactors, self.rh_th_center_tactors]

        self.lh_th_center_tactors = [10061, 10071, 10081, 10031, 10041, 10101, 10091]
        self.lh_ff_center_tactors = [11061, 11071, 11081, 11031, 11041, 11101, 11091]
        self.lh_mf_center_tactors = [12061, 12071, 12081, 12031, 12041, 12101, 12091]
        self.lh_rf_center_tactors = [13032, 13031, 13022, 13052]
        self.lh_lf_center_tactors = [14032, 14031, 14022, 14052]
        self.lh_center_tactors = [self.lh_ff_center_tactors, self.lh_mf_center_tactors, self.lh_rf_center_tactors, self.lh_lf_center_tactors, self.lh_th_center_tactors]
        
        # Peripheral tactors
        self.rh_th_peripheral_tactors = [11, 12, 13, 21, 51, 111, 121, 122, 123]
        self.rh_ff_peripheral_tactors = [1011, 1012, 1013, 1021, 1051, 1111, 1121, 1122, 1123]
        self.rh_mf_peripheral_tactors = [2011, 2012, 2013, 2021, 2051, 2111, 2121, 2122, 2123]
        self.rh_rf_peripheral_tactors = [3011, 3021, 3041, 3051, 3061]
        self.rh_lf_peripheral_tactors = [4011, 4021, 4041, 4051, 4061]
        self.rh_peripheral_tactors = [self.rh_ff_peripheral_tactors, self.rh_mf_peripheral_tactors, self.rh_rf_peripheral_tactors, self.rh_lf_peripheral_tactors, self.rh_th_peripheral_tactors]

        self.lh_th_peripheral_tactors = [1011, 1012, 1013, 1021, 1051, 10111, 10121, 10122, 10123]
        self.lh_ff_peripheral_tactors = [11011, 11012, 11013, 11021, 11051, 11111, 11121, 11122, 11123]
        self.lh_mf_peripheral_tactors = [12011, 12012, 12013, 12021, 12051, 12111, 12121, 12122, 12123]
        self.lh_rf_peripheral_tactors = [13011, 13021, 13041, 13051, 13061]
        self.lh_lf_peripheral_tactors = [14011, 14021, 14041, 14051, 14061]
        self.lh_peripheral_tactors = [self.lh_ff_peripheral_tactors, self.lh_mf_peripheral_tactors, self.lh_rf_peripheral_tactors, self.lh_lf_peripheral_tactors, self.lh_th_peripheral_tactors]

        # Maps finger topic to index of finger in tactor lists above
        self.finger_topic_map = {
            'right_hand/thumb_2_thumb_tip/force_torque_sensor': 4,
            'right_hand/index_3_index_tip/force_torque_sensor': 0,
            'right_hand/middle_3_middle_tip/force_torque_sensor': 1,
            'right_hand/ring_3_ring_tip/force_torque_sensor': 2,
            'right_hand/pinky_3_pinky_tip/force_torque_sensor': 3,
            'left_hand/thumb_2_thumb_tip/force_torque_sensor': 4,
            'left_hand/index_3_index_tip/force_torque_sensor': 0,
            'left_hand/middle_3_middle_tip/force_torque_sensor': 1,
            'left_hand/ring_3_ring_tip/force_torque_sensor': 2,
            'left_hand/pinky_3_pinky_tip/force_torque_sensor': 3
        }

        self.rh_contact_vals = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.lh_contact_vals = [0.0, 0.0, 0.0, 0.0, 0.0]

    def timer_callback(self):
        """Timer function for the Haptx Feedback node."""

        if self.use_right:
            tactor_msg = TactorState()
            tactor_msg.tactor_id = []
            tactor_msg.inflation = []

            for i in range(5):
                # Inflate center tactors fully
                tactor_msg.tactor_id += array('i', self.rh_center_tactors[i])
                tactor_msg.inflation += array('d', [1.0]*len(self.rh_center_tactors[i]))

                # Inflate peripheral tactor proportionally
                tactor_msg.tactor_id += array('i', self.rh_peripheral_tactors[i])
                tactor_msg.inflation += array('d', [self.rh_contact_vals[i]]*len(self.rh_peripheral_tactors[i]))

            self.rh_tactor_pub.publish(tactor_msg)

        if self.use_left:
            tactor_msg = TactorState()
            tactor_msg.tactor_id = []
            tactor_msg.inflation = []

            for i in range(5):
                # Inflate center tactors fully
                tactor_msg.tactor_id += array('i', self.lh_center_tactors[i])
                tactor_msg.inflation += array('d', [1.0]*len(self.lh_center_tactors[i]))

                # Inflate peripheral tactor proportionally
                tactor_msg.tactor_id += array('i', self.lh_peripheral_tactors[i])
                tactor_msg.inflation += array('d', [self.lh_contact_vals[i]]*len(self.lh_peripheral_tactors[i]))
        
            self.lh_tactor_pub.publish(tactor_msg)

    def contact_wrench_callback(self, msg):
        finger_frame = msg.header.frame_id
        finger_index = self.finger_topic_map[finger_frame]

        normal_force = -msg.wrench.force.z

        if normal_force > 0.01:
            if finger_frame[0] == 'r':
                self.rh_contact_vals[finger_index] = min(1.0, normal_force/0.98)
            else:
                self.lh_contact_vals[finger_index] = min(1.0, normal_force/0.98)


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
