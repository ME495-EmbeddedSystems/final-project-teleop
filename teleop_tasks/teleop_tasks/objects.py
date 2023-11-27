import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, WrenchStamped, Vector3
from teleop_interfaces.msg import FingerWrenches
from tf2_msgs.msg import TFMessage
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformStamped, TransformBroadcaster
from tf2_ros.buffer import Buffer
import numpy as np
import math

class Objects(Node):
    def __init__(self):
        super().__init__("objects")
        # Creating a timer for this node
        self.timer = self.create_timer(1/75, self.timer_callback)
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber for the object pose topic
        self.objSub = self.create_subscription(TFMessage, "/object/poses", self.getObjPose, 10)
        
        # Subscriber to get the wrenches
        self.objSub = self.create_subscription(WrenchStamped, "/fingertip/force_torque", self.getWrenches, 10)   
        
        # Publisher for a obejcts topic
        self.locationPub = self.create_publisher(Pose, "/objects", 10)
        self.leftWrenchPub = self.create_publisher(FingerWrenches, "/left_hand/fingertip_wrenches", 10)
        self.rightWrenchPub = self.create_publisher(FingerWrenches, "/right_hand/fingertip_wrenches", 10)
        
        # Global wrench arrays that get reset after being published
        self.leftWrenchArray = []
        self.leftFingerWrenches = FingerWrenches()
        self.rightWrenchArray = []
        self.rightFingerWrenches = FingerWrenches()
        
        # Defining deafult finger link mass
        self.fingerLinkMass = 0.1
        
    def getWrenches(self, msg=WrenchStamped):
        # Adds wrenches to buffers that are then published as an array with all
        # other forces belonging to that hand
        if msg.header.frame_id[0] == 'l':
            # Left hand wrench
            if len(self.leftWrenchArray) <= 4:
                self.leftWrenchArray.append(self.removeGravity(msg))
        if msg.header.frame_id[0] == 'r':
            # Left hand wrench
            if len(self.rightWrenchArray) <= 4:
                self.rightWrenchArray.append(self.removeGravity(msg))
        
        
    def getObjPose(self, msg):
        print(msg.transforms[1])
        
        """ This code is used to find out which of the transforms is the block
        counter = 0
        j = TransformStamped()
        for i in msg.transforms:
            j = i
            self.get_logger().info(counter)
            self.get_logger().info(j)
            self.get_logger().info()
            counter += 1"""
        
    def timer_callback(self):
        # Need to braodcast world so right and left hands have a fixed frame
        lh_world = TransformStamped()

        lh_world.header.stamp = self.get_clock().now().to_msg()
        lh_world.header.frame_id = 'world'
        lh_world.child_frame_id = 'left_hand/world'
        lh_world.transform.translation.x = 0.0
        lh_world.transform.translation.y = 0.0
        lh_world.transform.translation.z = 0.0
        lh_world.transform.rotation.x = 0.0
        lh_world.transform.rotation.y = 0.0
        lh_world.transform.rotation.z = 0.0
        lh_world.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(lh_world)
        
        rh_world = TransformStamped()

        rh_world.header.stamp = self.get_clock().now().to_msg()
        rh_world.header.frame_id = 'world'
        rh_world.child_frame_id = 'right_hand/world'
        rh_world.transform.translation.x = 0.0
        rh_world.transform.translation.y = 0.0
        rh_world.transform.translation.z = 0.0
        rh_world.transform.rotation.x = 0.0
        rh_world.transform.rotation.y = 0.0
        rh_world.transform.rotation.z = 0.0
        rh_world.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(rh_world)
        
        # Get the location of the object from the gazebo topic
        
        # Publish the location of the object in question at the rate required
        #self.locationPub.publish()
        
        # Once the Wrench arrays get to length 5 publish them
        if len(self.leftWrenchArray) == 5:
            self.leftFingerWrenches.finger_wrench = self.leftWrenchArray
            self.leftWrenchPub.publish(self.leftFingerWrenches)
            
            # Clear the wrench array
            self.leftWrenchArray.clear()
        if len(self.rightWrenchArray) == 5:
            self.rightFingerWrenches.finger_wrench = self.rightWrenchArray
            self.rightWrenchPub.publish(self.rightFingerWrenches)
            
            # Clear the wrench array
            self.rightWrenchArray.clear()
        
    def removeGravity(self, msg=WrenchStamped):
        # Setting the final force vector equal to the original
        msgNoGrav = msg
        
        # Figure out what finger this force is on
        curFingerTip = getFingertipFrameName(msg)
        
        # Grab the transform from world to the msg frame
        TfwQuat = self.tf_buffer.lookup_transform(curFingerTip, "world", rclpy.time.Time()).transform.rotation
        
        # Convert Tfw from quaternians to euler angles
        Rfw = quatToRot(TfwQuat.x, TfwQuat.y, TfwQuat.z, TfwQuat.w)
        
        Fgw = self.fingerLinkMass*np.array([0.0, 0.0, -9.8])
        Fgf = (Rfw @ Fgw)
        print(Rfw)
        gfVec = Vector3(x=Fgf[0], y=Fgf[1], z=Fgf[2])
        
        msgNoGrav.wrench.force.x = msg.wrench.force.x - gfVec.x
        msgNoGrav.wrench.force.y = msg.wrench.force.y - gfVec.y
        msgNoGrav.wrench.force.z = msg.wrench.force.z - gfVec.z
        
        return msgNoGrav   
    
def getFingertipFrameName(Wrenchmsg=WrenchStamped):
    if Wrenchmsg.header.frame_id[0] == 'l':
        # Left hand frame check index 10 for finger name
        match Wrenchmsg.header.frame_id[10]:
            case 't':
                return "left_hand/thumb_tip"
            case 'i':
                return "left_hand/index_tip"
            case 'm':
                return "left_hand/middle_tip"
            case 'r':
                return "left_hand/ring_tip"
            case 'p':
                return "left_hand/pinky_tip"
    if Wrenchmsg.header.frame_id[0] == 'r':
        # Left hand frame check index 11 for finger name
        match Wrenchmsg.header.frame_id[11]:
            case 't':
                return "right_hand/thumb_tip"
            case 'i':
                return "right_hand/index_tip"
            case 'm':
                return "right_hand/middle_tip"
            case 'r':
                return "right_hand/ring_tip"
            case 'p':
                return "right_hand/pinky_tip"
         
    return ""

def quatToRot(q0, q1, q2, q3):     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def main(args=None):
    """Objects' main function."""
    rclpy.init(args=args)
    node = Objects()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
