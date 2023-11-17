#! /usr/bin/env python3
from typing import Any
import rclpy
from rclpy.node import Node as RosNode

from sensor_msgs.msg import Image as RosImage, CompressedImage

from cv_bridge import CvBridge
import dataclasses
import cv2

import typing
import functools
import numpy as np
import dataclass_wizard


@dataclasses.dataclass
class FilterBase(dataclass_wizard.JSONFileWizard):

    def get_field_range(self, field_name: str) -> tuple[float, float]:
        """Get the possible min and max of a field

        Args:
            field_name (str): string name of one of the class's field

        Raises:
            ValueError: if the name of the field is not in the class
        """
        if field_name not in [f.name for f in dataclasses.fields(self)]:
            raise ValueError(
                f"Trying to get the range of a field: {field_name} that doesn't exists in data class"
            )

        # Have a strong coupling of the namings of variables
        return (getattr(self,
                        field_name.split("_")[0].upper() + "_MAX"),
                getattr(self,
                        field_name.split("_")[0].upper() + "_MIN"))

    def update_field_by_name(self, field_name: str, value: float) -> list[str]:
        x_max, x_min = self.get_field_range(field_name)
        value = min(x_max, max(value, x_min))
        setattr(self, field_name, value)
        return []

    def apply_filter(self, bgr_image):
        return bgr_image

    def ensure_linked_field_smaller(self, field, linked_field, margin=1):
        if ((getattr(self, field) - getattr(self, linked_field)) < margin):
            # also update counter part
            setattr(self, linked_field, getattr(self, field) - margin)
            return True
        return False

    def ensure_linked_field_larger(self, field, linked_field, margin=1):
        if ((getattr(self, linked_field) - getattr(self, field)) < margin):
            # also update counter part
            setattr(self, linked_field, getattr(self, field) + margin)
            return True
        return False


@dataclasses.dataclass
class HSVFilter(FilterBase):

    h_high: float = 179
    h_low: float = 0
    s_high: float = 255
    s_low: float = 60
    v_high: float = 250
    v_low: float = 40

    # The exact names of these const are referenced later.
    H_MAX: typing.ClassVar[float] = 179
    H_MIN: typing.ClassVar[float] = 0
    S_MAX: typing.ClassVar[float] = 255
    S_MIN: typing.ClassVar[float] = 0
    V_MAX: typing.ClassVar[float] = 255
    V_MIN: typing.ClassVar[float] = 0

    def update_field_by_name(self, field_name: str, value: float) -> list[str]:
        super().update_field_by_name(field_name, value)

        # The custom linked attribute updating
        channel, side = field_name.split("_")
        if side == "high":
            counter_part = channel + "_low"
            if (getattr(self, counter_part) > getattr(self, field_name)):
                # also update counter part
                setattr(self, counter_part, value)
        else:
            counter_part = channel + "_high"
            if (getattr(self, counter_part) < getattr(self, field_name)):
                # also update counter part
                setattr(self, counter_part, value)
        return [counter_part]

    def apply_filter(self, bgr_image):
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv_image, (self.h_low, self.s_low, self.v_low),
                           (self.h_high, self.s_high, self.v_high))


@dataclasses.dataclass
class BlobDetector(FilterBase):
    # open_size:float = 0

    maxThreshold: int = 101
    minThreshold: int = 11

    maxArea: int = 5999
    minArea: int = 601

    MAXTHRESHOLD_MAX: typing.ClassVar[float] = 250
    MAXTHRESHOLD_MIN: typing.ClassVar[float] = 30

    MINTHRESHOLD_MAX: typing.ClassVar[float] = 230
    MINTHRESHOLD_MIN: typing.ClassVar[float] = 2

    MAXAREA_MAX: typing.ClassVar[float] = 10000
    MAXAREA_MIN: typing.ClassVar[float] = 1500

    MINAREA_MAX: typing.ClassVar[float] = 3000
    MINAREA_MIN: typing.ClassVar[float] = 100

    # Overriding init for dataclass is bad
    def __post_init__(self):
        super().__init__()

        self.params = cv2.SimpleBlobDetector_Params()

        self.detector = cv2.SimpleBlobDetector_create(self.params)

    def update_field_by_name(self, field_name: str, value: float) -> list[str]:

        super().update_field_by_name(field_name, value)

        setattr(self.params, field_name, getattr(self, field_name))
        self.detector = cv2.SimpleBlobDetector_create(self.params)

        if field_name == "maxThreshold":
            if self.ensure_linked_field_smaller(field_name, "minThreshold", 2):
                return ["minThreshold"]

        if field_name == "minThreshold":
            if self.ensure_linked_field_larger(field_name, "maxThreshold", 2):
                return ["maxThreshold"]

        if field_name == "maxArea":
            if self.ensure_linked_field_smaller(field_name, "minArea", 10):
                return ["minArea"]

        if field_name == "minArea":
            if self.ensure_linked_field_larger(field_name, "maxArea", 10):
                return ["maxArea"]
        return []

    def apply_filter(self, bgr_image):
        keypoints = self.detector.detect(bgr_image)

        for keypoint in keypoints:
            x, y = keypoint.pt
            x = int(x)
            y = int(y)
            print(f"have keypoint at:{x} , {y}")
            print(f"angle {keypoint.angle}")

            cv2.circle(bgr_image, (x, y), 5, (100, 100, 255), -1)

        im_with_keypoints = cv2.drawKeypoints(bgr_image, keypoints, np.array([]), (0, 0, 255),
                                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return im_with_keypoints , keypoints


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

        self.blob_detector = BlobDetector()
        print(self.blob_detector)

        self.img_subscriber = self.create_subscription(RosImage, "/image_rect_color",
                                                       self.img_recv_callback, 10)
        self.br = CvBridge()
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)

        self.setup_cv_trackbar(self.hsv_filter)
        self.setup_cv_trackbar(self.blob_detector)

    # Yes, Intentionally stuff everything into the callback.
    # Color picking only need to update when we have new image anyway
    def img_recv_callback(self, msg: RosImage):
        # print(f" Header  {msg.header} Encoding  {msg.encoding} \n"
        # +f"Height {msg.height} Width  {msg.width} Step  {msg.step}")
        # Convert the message to a new image
        bgr_input_image = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        hsv_mask = self.hsv_filter.apply_filter(bgr_input_image)
        masked_image = cv2.bitwise_and(bgr_input_image, bgr_input_image, mask=hsv_mask)

        blob_marked = self.blob_detector.apply_filter(masked_image)

        stacked_image = np.hstack([bgr_input_image, blob_marked])
        scaled_image = self.scale_image_down(stacked_image)
        cv2.imshow(self.WINDOW_NAME, scaled_image)
        cv2.waitKey(50)
        # TODO
        # self.hsv_filter.to_json_file()

    def scale_image_down(self, image: np.ndarray):
        height, width, _ = image.shape

        scale_down_factor = 1800 / width
        return cv2.resize(image, (0, 0), fx=scale_down_factor, fy=scale_down_factor)

    # Node: the order of arguments matter.
    def update_field_callback(self, filter: FilterBase, field_name: str, new_value):
        # In theory this is actually very very bad. But in practice it worked.
        # Problem with cv2 setTrackbarPos is it also trigger callback (instead of
        # just user hitting it)
        # So this would have cause a infinite loop
        # In practice, the callback is only called when value is changed.
        # So what will happen is a callback to change h_high will not re-trigger itself
        # internally. However this this h_high callback also change h_low, it will
        # trigger a h_low callback. Luckly, since value is set, setting h_low won't trigger
        # another h_high callback

        linked_attrs = filter.update_field_by_name(field_name, new_value)
        actual_set_value = getattr(filter, field_name)
        print(f"Setting {field_name} to {actual_set_value}")
        cv2.setTrackbarPos(field_name, self.WINDOW_NAME, actual_set_value)

        for linked_attr in linked_attrs:
            print(f"Also update linked attr {linked_attr}")
            linked_value = getattr(filter, linked_attr)
            cv2.setTrackbarPos(linked_attr, self.WINDOW_NAME, linked_value)

    def setup_cv_trackbar(self, filter: FilterBase):
        default_value_map = {}
        for field in dataclasses.fields(filter):
            name = field.name
            print(f"\n-------\n  Adding field {name}")
            field_max, field_min = filter.get_field_range(name)
            # Shouldn't use lambda here, as lambda creates links not copies
            # binded_callback = lambda value: self.update_field_callback(name,value)

            # Note this binding is in orders, from left to right.
            binded_callback = functools.partial(self.update_field_callback, filter, name)

            print(f"max min {field_max} , {field_min}\n{binded_callback}")

            # When creating trackbar with min bigger then 0, or max smaller then 0.
            # the creating process will call setTrackbar to snap it to limit, but
            # will trigger a callback. Thus
            # The default value needs to to be remembered.
            default_value_map[name] = getattr(filter, name)
            # print(f"callback {binded_callback}")
            cv2.createTrackbar(name, self.WINDOW_NAME, field_min, field_max, binded_callback)

        for field in dataclasses.fields(filter):
            # Must do this after all bars being done. Or the set trackbar could trigger
            # a callback which cause the linked_attr's trackbar (not existing) to be sets
            print(f"-- Initializeing {field.name} to {default_value_map[field.name]} ")
            cv2.setTrackbarPos(field.name, self.WINDOW_NAME, default_value_map[field.name])


if __name__ == '__main__':
    main()