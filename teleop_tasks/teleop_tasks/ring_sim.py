import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Point32, Quaternion
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from enum import Enum, auto
import numpy as np
import random
import time

class SimState(Enum):
    REST = auto(),
    GRASPED = auto(),
    FALLING = auto(),
    LOCKED = auto(),

class RingSim(Node):
    """
    Publishes transforms for all the rings.
    """

    def __init__(self):
        super().__init__('ring_sim')

        self.goalZ = {
                    'blue' : 0.1,
                    'green' : 0.15, 
                    'yellow' : 0.20, 
                    'orange' : 0.25,
                    'red' : 0.30 
        }

        self.period = 0.01

        # Create control loop timer based on frequency parameter
        self.timer = self.create_timer(self.period, self.timer_callback)

        # Create broadcasters
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Create Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ring_positions = [(0,0)]

        x, y = self.generate_ring_start_position()
        self.blue = Ring(x0=0.25,y0=0.0,name="blue",period=self.period)
        x, y = self.generate_ring_start_position()
        self.green = Ring(x0=x,y0=y,name="green",period=self.period)
        x, y = self.generate_ring_start_position()
        self.yellow = Ring(x0=x,y0=y,name="yellow",period=self.period)
        x, y = self.generate_ring_start_position()
        self.orange = Ring(x0=x,y0=y,name="orange",period=self.period)
        x, y = self.generate_ring_start_position()
        self.red = Ring(x0=x,y0=y,name="red",period=self.period)

        self.ringArray = [self.blue, self.green, self.yellow, self.orange, self.red]

        # Transform for red ring in world
        self.world_to_ring_base = TransformStamped()
        self.world_to_ring_base.header.frame_id = "sim/world"
        self.world_to_ring_base.child_frame_id = "sim/ring_base/base"

        # Create publisher for markers
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, "visualization_marker", markerQoS)

        # Publish table marker
        table = Marker()
        table.header.stamp = self.get_clock().now().to_msg()
        table.header.frame_id = 'sim/world'
        table.type = table.CUBE
        table.action = table.ADD
        table.pose.position.z = -0.25
        table.scale.x = 2.0
        table.scale.y = 2.0
        table.scale.z = 0.5
        table.color.r = 0.68
        table.color.g = 0.51
        table.color.b = 0.32
        table.color.a = 1.0
        table.lifetime.nanosec = 0
        table.frame_locked = True
        table.id = 0
        self.marker_pub.publish(table)

        self.object_frame_map = {'blue_ring': 'blue_center',
                                 'green_ring': 'green_center',
                                 'yellow_ring': 'yellow_center',
                                 'orange_ring': 'orange_center',
                                 'red_ring': 'red_center'}
        
        self.state = SimState.REST
        
    def timer_callback(self):
        #self.publish_world()
        self.world_to_ring_base.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.world_to_ring_base)
        
        for i in self.ringArray:
            self.get_logger().info(i.name+"'s state is:" + str(i.state))
            if(i.state == SimState.REST):
                i.acceleration = Point32()
                i.velocity = Point32()
                i.position.z = 0.0155
                if(self.ring_grasped(i.name)):
                    try:
                        trans = self.tf_buffer.lookup_transform(i.TF.child_frame_id, "left_hand/palm", rclpy.time.Time(nanoseconds=0))
                        i.state = SimState.GRASPED

                        #i.offset.x = trans.transform.translation.x
                        #i.offset.y = trans.transform.translation.y

                        i.acceleration = Point32()
                        i.velocity = Point32()

                    except Exception as error:
                        self.get_logger().info(str(error))
            if(i.state == SimState.GRASPED):
                if(self.ring_grasped(i.name)):
                    try:
                        trans = self.tf_buffer.lookup_transform("sim/world", "left_hand/palm", rclpy.time.Time(nanoseconds=0))
        
                        i.acceleration = Point32()
                        i.velocity = Point32()
                        i.position.x = trans.transform.translation.x 
                        i.position.y = trans.transform.translation.y
                        i.position.z = trans.transform.translation.z + 0.1155
                        i.TF.transform.rotation = trans.transform.rotation
                        i.last_rotation = trans.transform.rotation

                    except Exception as error:
                        self.get_logger().info(str(error))
                else:
                    i.state = SimState.FALLING

            if(i.state == SimState.FALLING):
                i.acceleration.z = -9.81

                if(self.ring_placed(i.name)):
                    i.state = SimState.LOCKED
                if(i.position.z <= 0.0155):
                    i.state = SimState.REST

            if(i.state == SimState.LOCKED):
                i.acceleration = Point32()
                i.velocity = Point32()
                i.position = Point32()
                i.position.z = self.goalZ[i.name]
                i.TF.transform.rotation = Quaternion()
            
            #if(i.position.z <= 0):
            #    i.state = SimState.REST
            
            i.update_state()
            i.TF.header.stamp = self.get_clock().now().to_msg()
            self.broadcaster.sendTransform(i.TF)

    """
    def publish_world(self):
        self.world_to_blue.header.stamp = self.get_clock().now().to_msg()
        self.world_to_green.header.stamp = self.get_clock().now().to_msg()
        self.world_to_yellow.header.stamp = self.get_clock().now().to_msg()
        self.world_to_orange.header.stamp = self.get_clock().now().to_msg()
        self.world_to_red.header.stamp = self.get_clock().now().to_msg()

        self.broadcaster.sendTransform(self.world_to_blue)
        self.broadcaster.sendTransform(self.world_to_green)
        self.broadcaster.sendTransform(self.world_to_yellow)
        self.broadcaster.sendTransform(self.world_to_orange)
        self.broadcaster.sendTransform(self.world_to_red)
        self.broadcaster.sendTransform(self.world_to_ring_base)
    """

    def ring_placed(self, name):
        try:
            trans = self.tf_buffer.lookup_transform("sim/"+name+"/center", "sim/ring_base/base", rclpy.time.Time(nanoseconds=0))

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            if(z < 0.5 and x*x + y*y < (0.3)**2):
                return True
            return False
        except Exception as error:
            self.get_logger().info(str(error))


    def ring_grasped(self, name):
        """ Calculates 1x5 bitmap of which fingers are in contact with the ring.
            If thumb and another finger are touching a ring, that ring is grabbed.
        """

        try:
            # Lookup tf from ring to fingertips
            thumb_tf = self.tf_buffer.lookup_transform("sim/"+name+"/center", "left_hand/thumb_tip", rclpy.time.Time(nanoseconds=0)) # Thumb
            index_tf = self.tf_buffer.lookup_transform("sim/"+name+"/center", "left_hand/index_tip", rclpy.time.Time(nanoseconds=0)) # Index
            middle_tf = self.tf_buffer.lookup_transform("sim/"+name+"/center", "left_hand/middle_tip", rclpy.time.Time(nanoseconds=0)) # Middle
            ring_tf = self.tf_buffer.lookup_transform("sim/"+name+"/center", "left_hand/ring_tip", rclpy.time.Time(nanoseconds=0)) # Ring
            pinky_tf = self.tf_buffer.lookup_transform("sim/"+name+"/center", "left_hand/pinky_tip", rclpy.time.Time(nanoseconds=0)) # Pinky

            fingers_in_contact = np.zeros(5)

            fingers_in_contact[0] = self.finger_contact(thumb_tf)
            fingers_in_contact[1] = self.finger_contact(index_tf)
            fingers_in_contact[2] = self.finger_contact(middle_tf)
            fingers_in_contact[3] = self.finger_contact(ring_tf)
            fingers_in_contact[4] = self.finger_contact(pinky_tf)

            #self.get_logger().info(str(np.sum(fingers_in_contact)))

            if (fingers_in_contact[0] == 1) or (np.sum(fingers_in_contact) >= 1):

                return True
        
        except Exception as error:
            self.get_logger().info(str(error))

        return False


    def finger_contact(self, ring_fingertip_tf):

        # Ring Dimensions
        ring_width = 0.25#0.118
        ring_height = 0.25#0.031

        # Check if z is within height range of ring
        if abs(ring_fingertip_tf.transform.translation.z) <= ring_height/2:

            dist_fingertip_ring = (ring_fingertip_tf.transform.translation.x**2 + ring_fingertip_tf.transform.translation.y**2)**0.5

            if dist_fingertip_ring <= ring_width/2:

                return 1
            
        return 0

    def generate_ring_start_position(self):
        x = random.uniform(-0.5, 0.5)
        y = random.uniform(-0.5, 0.5)

        while self.too_close(x,y):
            x = random.uniform(-0.5, 0.5)
            y = random.uniform(-0.5, 0.5)

        self.ring_positions += [(x,y)]

        return x, y

    def too_close(self, x, y):
        for pos in self.ring_positions:
            d = ((x - pos[0])**2 + (y - pos[1])**2)**0.5

            if d < 0.15:
                return True

        return False

class Ring():

    def __init__(self, x0, y0, name, period):
        self.TF = TransformStamped()
        self.period = period
        self.name = name
        self.state = SimState.REST
        self.TF.header.frame_id = "sim/world"
        self.TF.child_frame_id = "sim/"+name+"/center"
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

    
    def update_state(self):
        self.position.x = self.position.x + self.velocity.x * self.period + 0.5 * self.acceleration.x * self.period * self.period
        self.velocity.x = self.velocity.x + self.acceleration.x * self.period
        self.TF.transform.translation.x = self.position.x

        self.position.y = self.position.y + self.velocity.y * self.period + 0.5 * self.acceleration.y * self.period * self.period
        self.velocity.y = self.velocity.y + self.acceleration.y * self.period
        self.TF.transform.translation.y = self.position.y

        self.position.z = self.position.z + self.velocity.z * self.period + 0.5 * self.acceleration.z * self.period * self.period
        self.velocity.z = self.velocity.z + self.acceleration.z * self.period
        self.TF.transform.translation.z = self.position.z

    
def main(args=None):
    rclpy.init(args=args)

    ring_sim_node = RingSim()

    rclpy.spin(ring_sim_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ring_sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
