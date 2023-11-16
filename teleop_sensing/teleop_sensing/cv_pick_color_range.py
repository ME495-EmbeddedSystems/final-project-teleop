#! /usr/bin/env python3
from typing import Any
import rclpy
from rclpy.node import Node as RosNode

from sensor_msgs.msg import Image as RosImage , CompressedImage

from cv_bridge import CvBridge
import dataclasses
import cv2

import typing
import functools
import numpy as np
@dataclasses.dataclass
class HSVFilter():

    h_high:float= 179
    h_low:float= 0
    s_high:float= 255
    s_low:float= 0
    v_high:float= 255
    v_low:float= 0

    # The exact names of these const are referenced later.
    H_MAX: typing.ClassVar[float] = 179
    H_MIN: typing.ClassVar[float] = 0
    S_MAX: typing.ClassVar[float] = 255
    S_MIN: typing.ClassVar[float] = 0
    V_MAX: typing.ClassVar[float] = 255
    V_MIN: typing.ClassVar[float] = 0

    def get_field_range(self,field_name:str)->tuple[float,float]:
        """Get the possible min and max of a field

        Args:
            field_name (str): string name of one of the class's field

        Raises:
            ValueError: if the name of the field is not in the class
        """
        if field_name not in [f.name for f in dataclasses.fields(self)]:
            raise ValueError(f"Trying to get the range of a field: {field_name} that doesn't exists in data class")

        # Have a strong coupling of the namings of variables
        return (getattr(self,field_name[0].upper() +"_MAX"),getattr(self,field_name[0].upper() +"_MIN"))

    def update_field_by_name(self , field_name:str , value:float)->str:
        
        x_max , x_min = self.get_field_range(field_name)
        value = min(x_max , max(value , x_min))
        setattr(self, field_name, value)

        channel , side = field_name.split("_")
        if side == "high":
            counter_part = channel + "_low"
            if(getattr(self,counter_part) > getattr(self,field_name)):
                # also update counter part
                setattr(self,counter_part,value)
        else:
            counter_part = channel + "_high"
            if(getattr(self,counter_part) < getattr(self,field_name)):
                # also update counter part
                setattr(self,counter_part,value)
        return counter_part

    def apply_filter(self,bgr_image):
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv_image, (self.h_low, self.s_low, self.v_low),
                           (self.h_high, self.s_high, self.v_high))


def main(args=None):
    '''
    The main entry point. This combines entry method for both ros2 run through
    setup.py as well as direct file execution 
    '''
    # This allows verifying the entry point
    print(f'Starting at main of turtle control, given args: {args}')
    rclpy.init(args=args)
    filter_finder = CvColorFilterFinder()
    rclpy.spin(filter_finder)
    rclpy.shutdown()
    # cv2.destroyAllWindows()


class CvColorFilterFinder(RosNode):

    WINDOW_NAME = "HSV_Filter_Finder"

    def __init__(self) -> None:
        super().__init__("image_process")
        self.info = self.get_logger().info
        self.debug = self.get_logger().debug
        self.warn = self.get_logger().warning
        self.error = self.get_logger().error

        self.hsv_filter = HSVFilter()

        self.img_subscriber = self.create_subscription(RosImage
         , "/image_rect_color" , self.img_recv_callback , 10)
        self.br = CvBridge()
        cv2.namedWindow(self.WINDOW_NAME )

        self.setup_cv_trackbar()

    # Yes, Intentionally stuff everything into the callback.
    # Color picking only need to update when we have new image anyway
    def img_recv_callback(self , msg:RosImage):
        # print(f" Header  {msg.header} Encoding  {msg.encoding} \n"
        # +f"Height {msg.height} Width  {msg.width} Step  {msg.step}")
        # Convert the message to a new image
        bgr_input_image = self.br.imgmsg_to_cv2(msg , desired_encoding="bgr8")


        hsv_mask = self.hsv_filter.apply_filter(bgr_input_image)
        masked_image = cv2.bitwise_and(bgr_input_image,bgr_input_image , mask=hsv_mask)

        cv2.imshow(self.WINDOW_NAME, np.hstack([bgr_input_image , masked_image]) )
        cv2.waitKey(50)


    # Node: the order of arguments matter.
    def update_field_callback(self,field_name:str , new_value):
        # if field_name not in dataclasses.fields(self):
        #     raise ValueError("Trying to update a field that doesn't exists in data class")

        counter_part_name = self.hsv_filter.update_field_by_name(field_name, new_value)

        # In theory this is actually very very bad. But in practice it worked.
        # Problem with cv2 setTrackbarPos is it also trigger callback (instead of
        # just user hitting it)
        # So this would have cause a infinite loop
        # In practice, the callback is only called when value is changed.
        # So what will happen is a callback to change h_high will not re-trigger itself
        # internally. However this this h_high callback also change h_low, it will
        # trigger a h_low callback. Luckly, since value is set, setting h_low won't trigger 
        # another h_high callback

        actual_set_value = getattr(self.hsv_filter , field_name)
        cv2.setTrackbarPos(field_name,self.WINDOW_NAME, actual_set_value)

        counter_part_value = getattr(self.hsv_filter , counter_part_name)
        cv2.setTrackbarPos(counter_part_name,self.WINDOW_NAME, counter_part_value)

        print(f"Setting {field_name} to {actual_set_value}")


    def setup_cv_trackbar(self):

        def nothing(value):
            print(f"called {value}")
        cv2.createTrackbar("a" , self.WINDOW_NAME, 0,124,nothing)

        for field in dataclasses.fields(self.hsv_filter):
            name = field.name
            field_max , field_min = self.hsv_filter.get_field_range(name)
            # Shouldn't use lambda here, as lambda creates links not copies
            # binded_callback = lambda value: self.update_field_callback(name,value)

            # Note this binding is in orders, from left to right.
            binded_callback = functools.partial(self.update_field_callback, name)

            cv2.createTrackbar(name,self.WINDOW_NAME , field_min,
                           field_max, binded_callback)
            cv2.setTrackbarPos(name,self.WINDOW_NAME, getattr(self.hsv_filter , name))
            print(f"Added trackbar for {name}")




if __name__ == '__main__':
    main()
