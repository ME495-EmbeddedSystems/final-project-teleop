import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped, WrenchStamped
from tf2_msgs.msg import TFMessage

class Objects(Node):
    def __init__(self):
        super().__init__("objects")
        # Creating a timer for this node
        self.timer = self.create_timer(1/100, self.timer_callback)
        
        # Subscriber for the object pose topic
        self.objSub = self.create_subscription(TFMessage, "/object/poses", self.getObjPose, 10)       
        
        # Publisher for a obejcts topic
        self.locationPub = self.create_publisher(Pose, "/objects", 10)
        
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
        pass
        
def main(args=None):
    """Objects' main function."""
    rclpy.init(args=args)
    node = Objects()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
