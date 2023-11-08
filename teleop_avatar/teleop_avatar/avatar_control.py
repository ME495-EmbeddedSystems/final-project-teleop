"""
Set up arena and brick markers and moves brick.

Services:
  + grasp (teleop_interfaces/srv/Grasp) - Grasp a specific object
  + execute_trajectory (std_srvs/srv/ExecuteTrajectory) - Move the grasped object along the specified trajectory

"""
import rclpy
from rclpy.node import Node


class AvatarControl(Node):
    """Actuate the ABB Gofas and Shadow Hands."""

    def __init__(self):
        super().__init__('avatar_control')

        # Create timer
        timer_freq = 50  # Hz
        self.timer = self.create_timer(1/timer_freq, self.timer_callback)

    def timer_callback(self):
        """Timer function for the Avatar Control node."""
        pass


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
