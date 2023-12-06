import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy

from geometry_msgs.msg import Wrench
import time


@pytest.mark.rostest
def generate_test_description():
    in_out_action = Node(package="teleop_haptics",
                         executable="force_feedback",
                         )
    return (
        LaunchDescription([
            in_out_action,
            launch_testing.actions.ReadyToTest()
            ]),
        # These are extra parameters that get passed to the test functions
        {
            'in_out': in_out_action
        }
    )


class TestAppliedWrenchHz(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.msg_times = []
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        self.node.destroy_node()

    def counting_callback(self, msg):
        """Count the number of messages and time it took."""
        self.msg_times.append(time.time())

    def test_static_transform(self, launch_service, in_out, proc_output):
        """Test cmd_vel frequency."""
        self.node.create_subscription(
            Wrench, "applied_wrench", self.counting_callback, 100)
        self.start_time = time.time()
        while (time.time() - self.start_time < 5):
            rclpy.spin_once(self.node)

        assert round(len(self.msg_times) / (self.msg_times[-1]-self.msg_times[0]), 0) == 100
