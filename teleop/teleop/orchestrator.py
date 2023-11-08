"""
Set up arena and brick markers and moves brick.

Service Clients:
  + grasp (teleop_interfaces/srv/Grasp) - Grasp a specific object
  + execute_trajectory (std_srvs/srv/ExecuteTrajectory) - Move the grasped object along the specified trajectory

"""
import rclpy
from rclpy.node import Node
from teleop_interfaces import Grasp, ExecuteTrajectory


class Orchestrator(Node):
    """Orchestrates the teleoperation experience."""

    def __init__(self):
        super().__init__('orchestrator')

        # Create timer
        timer_freq = 50  # Hz
        self.timer = self.create_timer(1/timer_freq, self.timer_callback)

        # Create service client to grasp object
        self.grasp_client = self.create_client(Grasp, "grasp")
        self.grasp_client.wait_for_service(timeout_sec=10)

        # Create service client to execute_trajectory
        self.execute_client = self.create_client(ExecuteTrajectory, "execute_trajectory")
        self.execute_client.wait_for_service(timeout_sec=10)

    def timer_callback(self):
        """Timer function for the Orchestrator node."""
        pass


def main(args=None):
    rclpy.init(args=args)

    orchestrator_node = Orchestrator()

    rclpy.spin(orchestrator_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    orchestrator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
