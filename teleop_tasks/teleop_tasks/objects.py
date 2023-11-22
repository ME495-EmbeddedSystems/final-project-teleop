import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, WrenchStamped
from teleop_interfaces.msg import FingerWrenches
from tf2_msgs.msg import TFMessage
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import rclpy
import numpy as np

class Objects(Node):
    def __init__(self):
        super().__init__("objects")
        # Creating a timer for this node
        self.timer = self.create_timer(1/100, self.timer_callback)
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
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
            print(counter)
            print(j)
            print()
            counter += 1"""
        
    def timer_callback(self):
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
        Tfw = self.tf_buffer.lookup_transform(curFingerTip, "world", rclpy.time.Time()).transform.rotation
        
        # Convert Tfw from quaternians to euler angles
        #self.get_logger().info(Tfw)
        
        # gw -> gf
        # Tfw*gw = gf
        # msg.forces - m*gf = msgNoGrav
        gw = np.array([0.0, 0.0, -9.8])
        #gf = Tfw @ gw
        #self.get_logger().info(f"{curFingerTip}: {msg.wrench.force}")
        
        return msgNoGrav   
    
def getFingertipFrameName(Wrenchmsg=WrenchStamped):
    if Wrenchmsg.header.frame_id[0] == 'l':
        # Left hand frame check index 10 for finger name
        match Wrenchmsg.header.frame_id[10]:
            case 't':
                return "thumb_tip"
            case 'i':
                return "index_tip"
            case 'm':
                return "middle_tip"
            case 'r':
                return "ring_tip"
            case 'p':
                return "pinky_tip"
    if Wrenchmsg.header.frame_id[0] == 'r':
        # Left hand frame check index 11 for finger name
        match Wrenchmsg.header.frame_id[11]:
            case 't':
                return "thumb_tip"
            case 'i':
                return "index_tip"
            case 'm':
                return "middle_tip"
            case 'r':
                return "ring_tip"
            case 'p':
                return "pinky_tip"
         
    return ""

def main(args=None):
    """Objects' main function."""
    rclpy.init(args=args)
    node = Objects()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
