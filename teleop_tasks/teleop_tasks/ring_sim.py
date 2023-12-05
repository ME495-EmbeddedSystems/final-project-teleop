import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Point32, Quaternion, Wrench
from teleop_interfaces.srv import SetWrench
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool, Empty
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from enum import Enum, auto
import numpy as np
import random


class SimState(Enum):
    """Control states for the Finite State Machine."""

    REST = (auto(),)
    GRASPED = (auto(),)
    FALLING = (auto(),)
    LOCKED = (auto(),)


class RingSim(Node):
    """
    Simulate physics for the rings in the RVIZ enviroment.

    Publishers
    ----------
        - /visualization_marker (visualization_msgs/msg/Marker) : The table
            marker publisher
        - /tf (geometry_msgs/msg/TransformStamped) : The transforms of
                                                        the following:
                                                        - The ring base
                                                        - ALl rings
    Services
    --------
        - /reset_rings (std_srvs/srv/Empty) : Reset the ring position
        and states

    Clients
    -------
        - /grasped (std_srvs/srv/SetBool) : Enable or diasble the Haptx
        -/left_hand/set_applied_wrench (teleop_interfaces/srv/SetWrench) :
            The set the applied force
    """

    def __init__(self):
        super().__init__("ring_sim")

        self.goalZ = [0.04, 0.065, 0.09, 0.115, 0.14]

        self.period = 0.01
        self.stacked = 0
        self.grabbed = None

        # Create control loop timer based on frequency parameter
        self.timer = self.create_timer(self.period, self.timer_callback)

        # Create broadcasters
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Create Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ring_positions = [(0, 0)]

        self.blue_start = Point32(x=0.25, y=0.0)
        self.green_start = Point32(x=0.0, y=-0.25)
        self.yellow_start = Point32(x=0.0, y=0.25)
        self.orange_start = Point32(x=0.0, y=0.40)
        self.red_start = Point32(x=0.0, y=-0.40)

        self.start_positons = {
            "blue": self.blue_start,
            "green": self.green_start,
            "yellow": self.yellow_start,
            "orange": self.orange_start,
            "red": self.red_start,
        }

        self.blue = Ring(name="blue", period=self.period, mass=0.5)
        self.green = Ring(name="green", period=self.period, mass=1.0)
        self.yellow = Ring(name="yellow", period=self.period, mass=1.5)
        self.orange = Ring(name="orange", period=self.period, mass=2.0)
        self.red = Ring(name="red", period=self.period, mass=2.5)

        self.ringArray = [self.blue, self.green, self.yellow,
                          self.orange, self.red]

        self.reset_rings()

        # Transform for ring base in world
        self.world_to_ring_base = TransformStamped()
        self.world_to_ring_base.header.frame_id = "sim/world"
        self.world_to_ring_base.child_frame_id = "sim/ring_base/base"

        # Create publisher for markers
        markerQoS = QoSProfile(depth=10,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(
            Marker, "visualization_marker", markerQoS
        )

        # Publish table marker
        table = Marker()
        table.header.stamp = self.get_clock().now().to_msg()
        table.header.frame_id = "sim/world"
        table.type = table.CUBE
        table.action = table.ADD
        table.pose.position.z = -0.25
        table.scale.x = 0.6
        table.scale.y = 1.0
        table.scale.z = 0.5
        table.color.r = 0.68
        table.color.g = 0.51
        table.color.b = 0.32
        table.color.a = 1.0
        table.lifetime.nanosec = 0
        table.frame_locked = True
        table.id = 0
        self.marker_pub.publish(table)

        # Create service client to actuate HaptX Gloves
        self.grasped_client = self.create_client(SetBool, "grasped")
        self.grasped_client.wait_for_service(timeout_sec=10)
        self.force_client = self.create_client(
            SetWrench, "/left_hand/set_applied_wrench"
        )
        self.force_client.wait_for_service(timeout_sec=10)

        self.reset_service = self.create_service(
            Empty, "reset_rings", self.reset_callback
        )

    def timer_callback(self):
        """
        Timer function.

        Runs through the following loop for each ring in the ringArray.

            REST:
                - Ring is at rest on the table.
                - Checks if picked up.
            GRASPED:
                - Ring is in the hand
                - Moves with the hand, checks if dropped.
            FALLING:
                - Ring is falling
                - Checks if landed on the pole or the table.
            LOCKED:
                - Ring is locked on the pole.
        """
        self.world_to_ring_base.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.world_to_ring_base)

        for i in self.ringArray:
            if i.state == SimState.REST:
                i.acceleration = Point32()
                i.velocity = Point32()
                i.position.z = 0.0155
                i.TF.transform.rotation = Quaternion()
                if self.ring_grasped(i.name):
                    try:
                        trans = self.tf_buffer.lookup_transform(
                            i.TF.child_frame_id,
                            "left_hand/palm",
                            rclpy.time.Time(nanoseconds=0),
                        )
                        i.state = SimState.GRASPED
                        self.grabbed = i.name
                        self.haptics(True, i.mass)

                        # Probably not worth since ring should be centered in
                        #   the hand.
                        # i.offset.x = trans.transform.translation.x
                        # i.offset.y = trans.transform.translation.y

                        i.acceleration = Point32()
                        i.velocity = Point32()

                    except Exception as error:
                        self.get_logger().info(str(error))
            if i.state == SimState.GRASPED:
                if self.ring_grasped(i.name, SimState.GRASPED):
                    i.counter = 0
                    try:
                        trans = self.tf_buffer.lookup_transform(
                            "sim/world",
                            "left_hand/palm",
                            rclpy.time.Time(nanoseconds=0),
                        )

                        i.acceleration = Point32()
                        i.velocity = Point32()
                        i.position.x = trans.transform.translation.x
                        i.position.y = trans.transform.translation.y
                        i.position.z = trans.transform.translation.z - 0.0155
                        i.TF.transform.rotation = trans.transform.rotation
                        i.last_rotation = trans.transform.rotation

                    except Exception as error:
                        self.get_logger().info(str(error))
                else:
                    i.counter += 1
                    if i.counter > 3:
                        i.counter = 0
                        i.state = SimState.FALLING
                        self.grabbed = None
                        self.haptics(False, 0.0)

            if i.state == SimState.FALLING:
                i.acceleration.z = -9.81

                if self.ring_placed(i.name):
                    i.state = SimState.LOCKED
                    i.acceleration = Point32()
                    i.velocity = Point32()
                    i.position = Point32()
                    i.position.z = self.goalZ[self.stacked]
                    i.TF.transform.rotation = Quaternion()
                    self.stacked += 1
                if i.position.z <= 0.0155:
                    i.state = SimState.REST

            i.update_state()
            i.TF.header.stamp = self.get_clock().now().to_msg()
            self.broadcaster.sendTransform(i.TF)

    def ring_placed(self, name):
        """
        Check if the ring is placed on the pole.

        Args:
        ----
            - name (string) : The name of the ring being checked

        Returns
        -------
            - True if placed on the pole, False otherwise

        """
        try:
            trans = self.tf_buffer.lookup_transform(
                "sim/" + name + "/center",
                "sim/ring_base/base",
                rclpy.time.Time(nanoseconds=0),
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            if z <= self.goalZ[self.stacked] and x * x + y * y < (0.1) ** 2:
                return True
            return False
        except Exception as error:
            self.get_logger().info(str(error))

    def ring_grasped(self, name, state=SimState.REST):
        """
        Calculate 1x5 bitmap of which fingers are in contact with the ring.

        If thumb and another finger are touching a ring, that ring is grabbed.

        Args:
        ----
            name (string) : The name of the ring being picked up.

        Returns
        -------
            (bool) : True if the ring is "grabbed", False otherwise

        """
        try:
            # Lookup tf from ring to fingertips
            thumb_tf = self.tf_buffer.lookup_transform(
                "sim/" + name + "/center",
                "left_hand/thumb_tip",
                rclpy.time.Time(nanoseconds=0),
            )  # Thumb
            index_tf = self.tf_buffer.lookup_transform(
                "sim/" + name + "/center",
                "left_hand/index_tip",
                rclpy.time.Time(nanoseconds=0),
            )  # Index
            middle_tf = self.tf_buffer.lookup_transform(
                "sim/" + name + "/center",
                "left_hand/middle_tip",
                rclpy.time.Time(nanoseconds=0),
            )  # Middle
            ring_tf = self.tf_buffer.lookup_transform(
                "sim/" + name + "/center",
                "left_hand/ring_tip",
                rclpy.time.Time(nanoseconds=0),
            )  # Ring
            pinky_tf = self.tf_buffer.lookup_transform(
                "sim/" + name + "/center",
                "left_hand/pinky_tip",
                rclpy.time.Time(nanoseconds=0),
            )  # Pinky

            fingers_in_contact = np.zeros(5)

            fingers_in_contact[0] = self.finger_contact(thumb_tf)
            fingers_in_contact[1] = self.finger_contact(index_tf)
            fingers_in_contact[2] = self.finger_contact(middle_tf)
            fingers_in_contact[3] = self.finger_contact(ring_tf)
            fingers_in_contact[4] = self.finger_contact(pinky_tf)

            if state == SimState.GRASPED:
                # Condition for remaining picked up is one finger is
                #   touching the ring.
                return np.sum(fingers_in_contact) > 0
            elif (fingers_in_contact[0] == 1) or (np.sum(
                            fingers_in_contact) >= 2):
                # Condition for picking up is that the thumb and one other
                # finger is in contact with the ring
                return True and (self.grabbed == name or self.grabbed is None)

        except Exception as error:
            self.get_logger().info(str(error))

        return False

    def finger_contact(self, ring_fingertip_tf):
        """
        Check if the ring is in contact with the finger.

        Args:
        ----
            - ring_fingertip_tf (TransformStamped) : The tf from the ring to
                the fingertip

        Returns
        -------
            - 1 if in contact, 0 otherwise.

        """
        # Ring Dimensions
        ring_width = 0.15  # 0.118
        ring_height = 0.1  # 0.031

        # Check if z is within height range of ring
        if abs(ring_fingertip_tf.transform.translation.z) <= ring_height / 2:
            dist_fingertip_ring = (
                ring_fingertip_tf.transform.translation.x**2
                + ring_fingertip_tf.transform.translation.y**2
            ) ** 0.5

            if dist_fingertip_ring <= ring_width / 2:
                return 1

        return 0

    def haptics(self, boolean, mass):
        """
        Call for haptic feedback.

        Args:
        ----
            boolean (Bool) : True to activate Haptx, False to deactivate Haptx
            mass (float) : The z force to apply with the frankas.

        """
        grasped_req = SetBool.Request()
        grasped_req.data = boolean
        self.grasped_client.call_async(grasped_req)
        force_req = SetWrench.Request()
        wrench = Wrench()
        wrench.force.z = -9.81 * mass
        force_req.wrench = wrench
        self.force_client.call_async(force_req)

    def generate_ring_start_position(self):
        """Generate a random ring start position."""
        x = random.uniform(-0.5, 0.5)
        y = random.uniform(-0.5, 0.5)

        while self.too_close(x, y):
            x = random.uniform(-0.5, 0.5)
            y = random.uniform(-0.5, 0.5)

        self.ring_positions += [(x, y)]

        return x, y

    def too_close(self, x, y):
        """
        Check if a ring start position is too close to another ring.

        Args:
        ----
            - x (float) : The x position
            - y (float) : The y position

        Returns
        -------
            - True if too close, false otherwise

        """
        for pos in self.ring_positions:
            d = ((x - pos[0]) ** 2 + (y - pos[1]) ** 2) ** 0.5

            if d < 0.15:
                return True

        return False

    def reset_rings(self):
        """Reset ring position to start position."""
        for i in self.ringArray:
            i.state = SimState.REST
            i.set_position(self.start_positons[i.name])

    def reset_callback(self, request, response):
        """
        Reset rings callback.

        Args:
            - request (Empty.Request): An empty request
            - response (Empty.Response): An empty response

        Returns
        -------
            - response (Empty.Response): Response sent back

        """
        self.reset_rings()
        return response


class Ring:
    """The ring object."""

    def __init__(self, name, period, x0=0.0, y0=0.0, mass=1.5):
        self.TF = TransformStamped()
        self.period = period
        self.name = name
        self.mass = mass
        self.counter = 0
        self.state = SimState.REST
        self.TF.header.frame_id = "sim/world"
        self.TF.child_frame_id = "sim/" + name + "/center"

        self.position = Point32()
        self.position.x = x0
        self.position.y = y0
        self.position.z = 0.0155
        self.velocity = Point32()
        self.acceleration = Point32()

        self.offset = Point32()
        self.rotation = Quaternion()
        self.last_rotation = Quaternion()

        self.TF.transform.translation.x = self.position.x
        self.TF.transform.translation.y = self.position.y
        self.TF.transform.translation.z = self.position.z

    def set_position(self, p):
        """
        Set the position of the ring object.

        Args:
        ----
            p (Point32) : The position to set it to.

        """
        self.position = p

    def update_state(self):
        """Update the internal ring state according to kinematic equations."""
        self.position.x = (
            self.position.x
            + self.velocity.x * self.period
            + 0.5 * self.acceleration.x * self.period * self.period
        )
        self.velocity.x = self.velocity.x + self.acceleration.x * self.period
        self.TF.transform.translation.x = self.position.x

        self.position.y = (
            self.position.y
            + self.velocity.y * self.period
            + 0.5 * self.acceleration.y * self.period * self.period
        )
        self.velocity.y = self.velocity.y + self.acceleration.y * self.period
        self.TF.transform.translation.y = self.position.y

        self.position.z = (
            self.position.z
            + self.velocity.z * self.period
            + 0.5 * self.acceleration.z * self.period * self.period
        )
        self.velocity.z = self.velocity.z + self.acceleration.z * self.period
        self.TF.transform.translation.z = self.position.z


def main(args=None):
    """Node's main function."""
    rclpy.init(args=args)
    ring_sim_node = RingSim()
    rclpy.spin(ring_sim_node)
    ring_sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
