import rclpy
from rclpy.node import Node as RosNode

from sensor_msgs.msg import Image as RosImage , CompressedImage

from cv_bridge import CvBridge

import cv2


def main(args=None):
    '''
    The main entry point. This combines entry method for both ros2 run through
    setup.py as well as direct file execution 
    '''
    # This allows verifying the entry point
    print(f'Starting at main of turtle control, given args: {args}')
    rclpy.init(args=args)
    image_processer = ImageProcesser()
    rclpy.spin(image_processer)
    rclpy.shutdown()



class ImageProcesser(RosNode):

    def __init__(self) -> None:
        super().__init__("image_process")
        self.info = self.get_logger().info
        self.debug = self.get_logger().debug
        self.warn = self.get_logger().warning
        self.error = self.get_logger().error

        self.img_subscriber = self.create_subscription(RosImage
         , "/image_rect_color" , self.img_recv_callback , 10)
        self.br = CvBridge()
        
    def img_recv_callback(self , msg:RosImage):
        print(f"""
Header  {msg.header}")
Encoding  {msg.encoding}
Height  {msg.height}
Width  {msg.width}
Step  {msg.step}
""")        
        # Convert the message to a new image
        im2 = self.br.imgmsg_to_cv2(msg , desired_encoding="bgr8") 
        # print(f"Shape of compressed {im2}")
        cv2.imshow("image", im2)
        cv2.waitKey(50)

if __name__ == '__main__':
    main()
