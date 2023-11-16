#! /usr/bin/env python3
import collections
import dataclasses
import functools

import cv2
import numpy as np
# Copied from example.
import pyrealsense2 as rs2
import rclpy
from cv_bridge import CvBridge
from cv_pick_color_range import BlobDetector, FilterBase, HSVFilter
from rclpy.node import Node as RosNode
from rclpy.time import Time as RosTime
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import CameraInfo, CompressedImage
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped , Transform , Vector3 , Point as PointMsg

if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

from tf2_ros import TransformBroadcaster


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
    WINDOW_NAME = "detection window"

    class TimedKeypoints():
        time_stamp:float
        keypoints :list

    

    def __init__(self) -> None:
        super().__init__("image_process")
        self.info = self.get_logger().info
        self.debug = self.get_logger().debug
        self.warn = self.get_logger().warning
        self.error = self.get_logger().error
        self.info("Camera info is setup. Assume it never changes")
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.hsv_filter = HSVFilter()
        self.blob_detector = BlobDetector()

        self.setup_cv_trackbar(self.blob_detector)
        self.setup_cv_trackbar(self.hsv_filter)

        self.br = CvBridge()

        self.depth_topic = "/D435i/aligned_depth_to_color/image_raw"

        self.intrinsics = self.record_camera_info_once()

        self.timed_keypoints = ImageProcesser.TimedKeypoints()
        # store 5 image to past
        self.depth_image_deque = collections.deque(maxlen= 3)

        # self.depth_image_sub = self.create_subscription(RosImage, "/D435i/depth/image_rect_raw", self.depth_image_cb,10)
        self.depth_image_sub = self.create_subscription(RosImage, self.depth_topic, self.depth_image_cb,10)
        self.img_subscriber = self.create_subscription(RosImage, "/image_rect_color",self.color_img_recv_callback, 10)

        # self.process_timer = self.create_timer(1/50 ,self.find_ring_process_timercb)

        self.transform_broadcaster = TransformBroadcaster(self)

    def color_img_recv_callback(self, msg: RosImage):

        bgr_input_image = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        hsv_mask = self.hsv_filter.apply_filter(bgr_input_image)

        kernel = np.ones((5,5),np.uint8)
        closed_mask = cv2.morphologyEx(hsv_mask, cv2.MORPH_CLOSE, kernel)
        opened_mask = cv2.morphologyEx(closed_mask, cv2.MORPH_OPEN, kernel)
        masked_image = cv2.bitwise_and(bgr_input_image, bgr_input_image, mask=opened_mask)

        blob_marked , keypoints = self.blob_detector.apply_filter(masked_image)
        
        cv2.imshow(self.WINDOW_NAME, self.scale_image_down(blob_marked , 900))
        cv2.waitKey(50)

        if not keypoints :
            return

        # Can't do anything without an image
        if len(self.depth_image_deque) < 1 :
            return
        # Try to map it to depth image if there exists keypoints
        
        
        keypoint_time = RosTime.from_msg(msg.header.stamp).nanoseconds * 1e-9
            
         
        depth_image = self.depth_image_deque.pop()
        cv_depth_image = self.br.imgmsg_to_cv2(depth_image, depth_image.encoding)
        
        
        # Assume intrinsics are received
        i=0 
        for keypoint in keypoints:
            px, py = keypoint.pt
            px= int(px)
            py = int(py)

            depth = cv_depth_image[py,px]
            print(f"---\nAt pixel {px} ,{py}, Depth: {depth}")
            sx,sy,sz = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [px, py], depth)
            print(f"deprojected space loc is {sx} , {sy} , {sz} ")    

            tf = TransformStamped()
            tf.header.stamp = depth_image.header.stamp
            tf.header.frame_id = depth_image.header.frame_id
            tf.child_frame_id = f"keypoint_{i}"
            tf.transform.translation.x = sx/1000
            tf.transform.translation.y = sy/1000
            tf.transform.translation.z = sz/1000
            self.transform_broadcaster.sendTransform(tf)
            i+=1
        self.transform_broadcaster.sendTransform


    def record_camera_info_once(self) ->rs2.intrinsics:
        print(f"Waiting for camera info")
        have_msg , camera_info =wait_for_message(CameraInfo,self, "/D435i/aligned_depth_to_color/camera_info",time_to_wait=10)

        print(f"Got camera info")
        if have_msg is None:
            raise RuntimeError("Did not get camera info after a long wait")
            return
        intrinsics = rs2.intrinsics()
        intrinsics.width = camera_info.width
        intrinsics.height = camera_info.height
        intrinsics.ppx = camera_info.k[2]
        intrinsics.ppy = camera_info.k[5]
        intrinsics.fx = camera_info.k[0]
        intrinsics.fy = camera_info.k[4]
        if camera_info.distortion_model == 'plumb_bob':
            intrinsics.model = rs2.distortion.brown_conrady
        elif camera_info.distortion_model == 'equidistant':
            intrinsics.model = rs2.distortion.kannala_brandt4
        intrinsics.coeffs = [i for i in camera_info.d]

        return intrinsics

    def depth_image_cb(self, depth_image):
        self.depth_image_deque.appendleft(depth_image)

    def find_ring_process_timercb(self):
        pass



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

    def scale_image_down(self, image: np.ndarray , expected_width:float):
        height, width, _ = image.shape

        scale_down_factor = expected_width / width
        return cv2.resize(image, (0, 0), fx=scale_down_factor, fy=scale_down_factor)


if __name__ == '__main__':
    main()
