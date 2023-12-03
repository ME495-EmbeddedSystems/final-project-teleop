#! /usr/bin/env python3
import collections
import dataclasses
import functools
from typing import Optional

import cv2
import numpy as np
# Copied from example.
import pyrealsense2 as rs2

if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

import enum

import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from image_geometry import PinholeCameraModel
from rcl_interfaces.msg import ParameterDescriptor as RosParameterDescriptor
from rclpy.node import Node as RosNode
from rclpy.time import Time as RosTime
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Header
from teleop_sensing.cv_filers import (BlobDetector, HOnlyFilter, HSVFilter, HSVFilter_SVBase,
                                      RedHFilter, TrackBarHelper)
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer as TfBuffer
from tf2_ros.transform_listener import TransformListener


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


class Color(enum.Enum):
    RED = enum.auto()
    GREEN = enum.auto()
    BLUE = enum.auto()
    ORANGE = enum.auto()
    YELLOW = enum.auto()


@dataclasses.dataclass()
class ColoredFilter():
    color: Color
    filter: HOnlyFilter


class ImageProcesser(RosNode):

    def __init__(self) -> None:
        super().__init__("cv_ring_finding")
        self.info = self.get_logger().info
        self.debug = self.get_logger().debug
        self.warn = self.get_logger().warning
        self.error = self.get_logger().error

        self.declare_parameter(
            "debug", True,
            RosParameterDescriptor(description="launch debug cv2 windows when True."))

        self.debug_mode: float = self.get_parameter("debug").get_parameter_value().bool_value

        # Setup the user tuned filters
        # TODO(LEO) Take param to decide on seeing the trackbar
        self.trackbar_helper = TrackBarHelper()
        self.sv_filter = HSVFilter_SVBase(255, 125, 255, 115)
        self.blob_detector = BlobDetector(maxArea=6000, minArea=500)
        if self.debug_mode:
            self.trackbar_helper.setup_cv_trackbar(self.sv_filter, "S V shared for all")
            self.trackbar_helper.setup_cv_trackbar(self.blob_detector, "Blob param")

        # red have to sets of bars, which we took union of
        self.color_filter_map: dict[Color, HOnlyFilter | RedHFilter] = {
            Color.RED:
                RedHFilter(170, 5),
            Color.GREEN:
                HOnlyFilter(85, 35),
            Color.BLUE:
                HOnlyFilter(98, 85),
            # Color.ORANGE: HOnlyFilter(20,8),
            # Special treatment for orange. It's really close to yellow
            Color.ORANGE:
                HSVFilter(20, 8, 255, 160, 255, 140),
            Color.YELLOW:
                HOnlyFilter(30, 18),
        }

        # Go through all colors and setup trackbar
        if self.debug_mode:
            for c, trackbar in self.color_filter_map.items():
                self.trackbar_helper.setup_cv_trackbar(trackbar, c.name)

        self.br = CvBridge()

        # self.intrinsics = self.record_camera_info_once()
        self.transform_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = TfBuffer()
        self.tf_listener = TransformListener(self._tf_buffer, self)

        self.depth_image_deque = collections.deque(maxlen=2)
        self.depth_cam_info_deque = collections.deque(maxlen=2)
        self.raw_image_deque = collections.deque(maxlen=2)
        self.raw_cam_info_deque = collections.deque(maxlen=2)

        self.named_queue = {
            "depth_image": self.depth_image_deque,
            "depth_cam_info": self.depth_cam_info_deque,
            "raw_image": self.raw_image_deque,
            "raw_cam_info": self.raw_cam_info_deque,
        }

        # Setup apriltags

        # /D435i/aligned_depth_to_color/image_raw
        # The non alighted depth image is not usable. points near the corner of the image will have huge bad offset.
        self.depth_image_sub = self.create_subscription(
            RosImage, "/D435i/aligned_depth_to_color/image_raw",
            functools.partial(self.push_to_queue_callback, self.depth_image_deque), 10)
        self.depth_ci_sub = self.create_subscription(
            CameraInfo, "/D435i/aligned_depth_to_color/camera_info",
            functools.partial(self.push_to_queue_callback, self.depth_cam_info_deque), 10)

        self.raw_image_sub = self.create_subscription(
            RosImage, "/D435i/color/image_raw",
            functools.partial(self.push_to_queue_callback, self.raw_image_deque), 10)
        self.raw_ci_sub = self.create_subscription(
            CameraInfo, "/D435i/color/camera_info",
            functools.partial(self.push_to_queue_callback, self.raw_cam_info_deque), 10)

        # intentionally to be between 1~2 times the image publishing rate. So the timer is likely in between when all image topics
        # has done publishing.
        self.process_timer = self.create_timer(1 / 50, self.find_ring_process_time_cb)
        print("Done Initializing")

    def push_to_queue_callback(self, target_queue: collections.deque, msg):
        target_queue.append(msg)

    def find_ring_process_time_cb(self):
        # Lets go through all the dequeue to check frame alignment.
        named_time = {}
        for name, queue in self.named_queue.items():
            if len(queue) < 1:
                # No item in the queue
                # print(f"{name} queue doesn't have data to use!")
                return
            # peek left
            obj = queue[0]
            named_time[name] = RosTime.from_msg(obj.header.stamp).nanoseconds * 1e-9

        base_name, base_time = list(named_time.items())[0]

        # print(f"Comparing to {base_name} at {base_time}")

        # TODO (LEO) Only operate at prefect timing, which actually should be common
        for name, time in named_time.items():
            d_time = time - base_time
            if d_time > (1 / 20):
                print(
                    f"{name} is {d_time} behind {base_name}! Above the 1 / 20(fps) limit Skipping cycle due to this"
                )
                return
            # print(f"{name} , {time-base_time}")
        """
        
        Conclusion with all the different route of getting image and depth 

        Getting aligned image: 
        The rectified output from image_proc nodes produced the same output as 
        Using camera info and feed raw image through PinholeCameraModel.  
        With added benefit of no need to rectify entire image, can rect only a point.

        Getting space xyz from pixel uv - aka de-projection:
        Converting the camera info into Realsense library intrinsics give the 
        same result as PinholeCameraModel (projectPixelTo3dRay * depth)

        Regardless of the method chosen from the above methods. The resulted space xyz 
        point is still offset to the side. After applying the same offset as 
        realsense_optical to realsense_depth frames, the offset is gone.

        Note: however the depth gathered from that point is still at the offseted location, 
        Which mean applying the fix offset at the end is kinda a hack

        Maybe the rs_ros library labeled the frame backwords?  
        """

        raw_img = self.raw_image_deque.pop()
        raw_ci = self.raw_cam_info_deque.pop()
        depth_img = self.depth_image_deque.pop()
        depth_ci = self.depth_cam_info_deque.pop()

        # TODO what happen with encoding =  image.encode
        raw_img_cv = self.br.imgmsg_to_cv2(raw_img, desired_encoding="bgr8")
        depth_img_cv = self.br.imgmsg_to_cv2(depth_img, depth_img.encoding)

        # This is stuff for the raw image:
        raw_pin_model = PinholeCameraModel()
        raw_pin_model.fromCameraInfo(raw_ci)

        # First apply the SV filter
        sv_mask: np.ndarray = self.sv_filter.apply_filter(raw_img_cv)

        # Now finish the rest with a per-color process

        # color_loc :dict[Color,tuple[float,float,float]] ={}

        for c, h_filter in self.color_filter_map.items():
            # This is a mask not allowing anything.
            # We then open up the masks from segment of ranges of colors.
            color_mask = h_filter.apply_filter(raw_img_cv)

            full_hsv_mask = cv2.bitwise_and(sv_mask, color_mask)

            raw_blob_marked, raw_pxy = self.find_donute_with_color_mask(raw_img_cv, full_hsv_mask)
            if self.debug_mode:
                cv2.imshow(c.name, self.trackbar_helper.scale_image_down(raw_blob_marked, 700))
                cv2.waitKey(1)

            if not raw_pxy:
                continue
                # Skip this color if nothing detected
            raw_rect_pxy = raw_pin_model.rectifyPoint(raw_pxy)
            raw_rect_pxy = [int(x) for x in raw_rect_pxy]
            raw_rect_depth = depth_img_cv[raw_rect_pxy[1], raw_rect_pxy[0]]
            # print(f"Color {c.name} depth {raw_rect_depth}")
            raw_rect_ph_sxyz = [
                v * raw_rect_depth / 1000 for v in raw_pin_model.projectPixelTo3dRay(raw_rect_pxy)
            ]
            print(
                f"Color {c.name} at xyz {raw_rect_ph_sxyz[0]:.5f} {raw_rect_ph_sxyz[1]:.5f} {raw_rect_ph_sxyz[2]:.5f} "
            )

            self.publish_circle_tf(depth_img.header, f"{c.name.lower()}_center", raw_rect_ph_sxyz)

    def find_donute_with_color_mask(self, bgr_image, color_mask):

        kernel = np.ones((3, 3), np.uint8)
        closed_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)
        opened_mask = cv2.morphologyEx(closed_mask, cv2.MORPH_OPEN, kernel)
        masked_image = cv2.bitwise_and(bgr_image, bgr_image, mask=opened_mask)

        blob_marked, keypoints = self.blob_detector.apply_filter(masked_image)

        pxy = None
        if keypoints:
            pxy = self.keypoint_to_pxy(self.get_largest_keypoint(keypoints))

        return blob_marked, pxy

    def find_donute_process(self, bgr_image: np.ndarray):
        hsv_mask = self.hsv_filter.apply_filter(bgr_image)
        return self.find_donute_with_color_mask(bgr_image, hsv_mask)

    def get_largest_keypoint(self, keypoints: list):
        max_size = 0
        max_key = None
        for key in keypoints:
            if key.size > max_size:
                max_size = key.size
                max_key = key
        return max_key

    def keypoint_to_pxy(self, keypoint):
        px, py = keypoint.pt
        px = int(px)
        py = int(py)
        return px, py

    def publish_circle_tf(self, image_header: Header, child_frame_id: str, sxyz: list[float]):
        tf = TransformStamped()
        tf.header.stamp = image_header.stamp
        tf.header.frame_id = image_header.frame_id
        tf.child_frame_id = child_frame_id
        tf.transform.translation.x = sxyz[0]
        tf.transform.translation.y = sxyz[1]
        tf.transform.translation.z = sxyz[2]
        print(f"Publishing {tf}")
        self.transform_broadcaster.sendTransform(tf)

    def depth_image_cb(self, depth_image):
        self.depth_image_deque.appendleft(depth_image)

    def get_transform(self, target_frame, source_frame) -> Optional[TransformStamped]:
        """Get the transform between two frame

        Args:
            target_frame (str): The target frame name
            source_frame (str): The source frame name

        Returns:
            Optional[TransformStamped]: None if error when getting tf. 
                TransformStamped between the frames given 
        """
        try:
            # get the latest transform between left and right
            # (rclpy.time.Time() means get the latest information)
            trans = self._tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return trans
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.error(f"Lookup exception: {e}")
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.error(f"Connectivity exception: {e}")
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.error(f"Extrapolation exception: {e}")
        return None


if __name__ == '__main__':
    main()
