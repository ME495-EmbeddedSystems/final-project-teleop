"""
Set up arena and brick markers and moves brick.

Service Clients:
  + grasp (teleop_interfaces/srv/Grasp) - Grasp a specific object
  + execute_trajectory (std_srvs/srv/ExecuteTrajectory) - Move the grasped
  object along the specified trajectory

"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from teleop_interfaces.srv import Grasp, ExecuteTrajectory
from enum import Enum


class TeleopState(Enum):
    WAIT = 0
    MOVING = 1


class Orchestrator(Node):
    """Orchestrates the teleoperation experience."""

    def __init__(self):
        super().__init__("orchestrator")

        self.cbgroup = ReentrantCallbackGroup()

        # Create timer
        timer_freq = 50  # Hz
        self.timer = self.create_timer(
            1 / timer_freq, self.timer_callback, self.cbgroup
        )

        # Create service client to grasp object
        self.grasp_client = self.create_client(Grasp, "grasp")
        self.grasp_client.wait_for_service(timeout_sec=10)

        # Create service client to execute_trajectory
        self.execute_client = self.create_client(
            ExecuteTrajectory, "execute_trajectory"
        )
        self.execute_client.wait_for_service(timeout_sec=10)

        self.object_ids = [
            "blue_ring",
            "green_ring",
            "yellow_ring",
            "orange_ring",
            "red_ring",
        ]
        self.last_tfs = {}

        self.state = TeleopState.WAIT
        self.moving_object_id = None

    async def timer_callback(self):
        """Timer function for the Orchestrator node."""
        if self.state == TeleopState.WAIT:
            for id in self.object_ids:
                world_object_tf = None  # Get transform

                # If transform for object already exists in dictionary
                if id in self.last_tfs:
                    # Check if object transform has changed
                    if self.last_tfs[id] is not world_object_tf:
                        self.get_logger().info(id + " is moving")

                        # If it has changed, grasp the object
                        await self.grasp(id)

                        # Update object's last known transform
                        self.last_tfs[id] = world_object_tf

                        self.moving_object_id = id
                        self.state = TeleopState.MOVING

                # Otherwise, set the initial transform
                else:
                    self.last_tfs[id] = world_object_tf

        if self.state == TeleopState.MOVING:
            world_object_tf = None  # Get transform

            # If object stops moving, execute trajectory
            if self.last_tfs[self.moving_object_id] == world_object_tf:
                req = ExecuteTrajectory.Request()
                req.object_id = self.moving_object_id
                self.execute_client.call_async(req)

                await self.execute_client.call_async(req)

                self.state = TeleopState.WAIT

    async def grasp(self, object_id):
        req = Grasp.Request()
        req.object_id = object_id
        self.grasp_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)

    orchestrator_node = Orchestrator()

    rclpy.spin(orchestrator_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    orchestrator_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
