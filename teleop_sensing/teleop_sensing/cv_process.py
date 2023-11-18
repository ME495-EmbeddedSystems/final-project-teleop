#! /usr/bin/env python3
import collections
import dataclasses
import functools
from typing import Optional

import cv2
import numpy as np
# Copied from example.
import pyrealsense2 as rs2
import rclpy
from cv_bridge import CvBridge
from teleop_sensing.cv_pick_color_range import BlobDetector, FilterBase, HSVFilter
from rclpy.node import Node as RosNode
from rclpy.time import Time as RosTime
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import CameraInfo, CompressedImage
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped , Transform , Vector3 , Point as PointMsg
from tf2_ros.buffer import Buffer as TfBuffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros

from image_geometry import PinholeCameraModel
from std_msgs.msg import Header
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
        super().__init__("cv_ring_finding")
        self.info = self.get_logger().info
        self.debug = self.get_logger().debug
        self.warn = self.get_logger().warning
        self.error = self.get_logger().error
        self.info("Camera info is setup. Assume it never changes")
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.hsv_filter = HSVFilter()
        self.blob_detector = BlobDetector()

        self.setup_cv_trackbar(self.hsv_filter)
        self.setup_cv_trackbar(self.blob_detector)
        cv2.imshow(self.WINDOW_NAME , np.zeros((1,1,3), np.uint8))
        cv2.waitKey(1)
        self.br = CvBridge()

        self.depth_topic = "/D435i/aligned_depth_to_color/image_raw"

        # self.intrinsics = self.record_camera_info_once()
        self.transform_broadcaster = TransformBroadcaster(self)

        self.timed_keypoints = ImageProcesser.TimedKeypoints()
        # store 5 image to past

        self.depth_image_deque = collections.deque(maxlen= 2)
        self.depth_cam_info_deque = collections.deque(maxlen= 2)

        self.raw_image_deque = collections.deque(maxlen= 2)
        self.raw_cam_info_deque = collections.deque(maxlen=2)

        self.rect_image_deque = collections.deque(maxlen= 2)
        # self.rect_cam_info_deque = collections.deque(maxlen= 2)

        self.named_queue = {
            "depth_image" : self.depth_image_deque , 
            "depth_cam_info" : self.depth_cam_info_deque , 
            "raw_image" : self.raw_image_deque , 
            "raw_cam_info" : self.raw_cam_info_deque , 
            "rect_image" : self.rect_image_deque
        }

        # /D435i/aligned_depth_to_color/image_raw
        # The non alighted depth image is not usable. points near the corner of the image will have huge bad offset. 
        # self.depth_image_sub = self.create_subscription(RosImage, "/D435i/depth/image_rect_raw", functools.partial(self.push_to_queue_callback, self.depth_image_deque) ,10)
        # self.depth_ci_sub = self.create_subscription(CameraInfo, "/D435i/depth/camera_info", functools.partial(self.push_to_queue_callback, self.depth_cam_info_deque) ,10)

        self.depth_image_sub = self.create_subscription(RosImage, "/D435i/aligned_depth_to_color/image_raw", functools.partial(self.push_to_queue_callback, self.depth_image_deque) ,10)
        self.depth_ci_sub = self.create_subscription(CameraInfo, "/D435i/aligned_depth_to_color/camera_info", functools.partial(self.push_to_queue_callback, self.depth_cam_info_deque) ,10)

        self.raw_image_sub = self.create_subscription(RosImage, "/D435i/color/image_raw", functools.partial(self.push_to_queue_callback, self.raw_image_deque) ,10)
        self.raw_ci_sub = self.create_subscription(CameraInfo, "/D435i/color/camera_info", functools.partial(self.push_to_queue_callback, self.raw_cam_info_deque) ,10)
        self.rect_image_sub = self.create_subscription(RosImage, "/image_rect_color", functools.partial(self.push_to_queue_callback, self.rect_image_deque) ,10)
        # self.depth_image_sub = self.create_subscription(RosImage, "/camera_info", functools.partial(self.push_to_queue_callback, self.rect_cam_info_deque) ,10)

        self._tf_buffer = TfBuffer()
        self.tf_listener = TransformListener(self._tf_buffer, self)


        self.process_timer = self.create_timer(1/50 ,self.find_ring_process_time_cb)
        print(f"Done init")

    def push_to_queue_callback(self,target_queue:collections.deque ,msg):
        target_queue.append(msg)

    def find_ring_process_time_cb(self):
        # Lets go through all the dequeue to check frame alignment.
        named_time = {}
        for name in self.named_queue:
            if len(self.named_queue[name]) <1:
                return
            # peek left
            obj = self.named_queue[name][0]
            named_time[name] = RosTime.from_msg(obj.header.stamp).nanoseconds * 1e-9

        base_name , base_time = list(named_time.items())[0]

        print(f"Comparing to {base_name} at {base_time}")

        # TODO (LEO) Only operate at prefect timing, which actually should be common
        for name, time in named_time.items():
            print(f"{name} , {time-base_time}")

        raw_img = self.raw_image_deque[0]
        raw_ci = self.raw_cam_info_deque[0]
        depth_img = self.depth_image_deque[0]
        depth_ci = self.depth_cam_info_deque[0]
        rect_img = self.rect_image_deque[0]


        # TODO what happen with encoding =  image.encode
        raw_img_cv = self.br.imgmsg_to_cv2( raw_img, desired_encoding="bgr8")
        depth_img_cv = self.br.imgmsg_to_cv2(depth_img, depth_img.encoding)
        rect_img_cv = self.br.imgmsg_to_cv2( rect_img, desired_encoding="bgr8")

        # See what happen when I rect the depth.
        depth_pin_model = PinholeCameraModel()
        depth_pin_model.fromCameraInfo(depth_ci)
        depth_rect_img_cv = depth_img_cv
        # depth_pin_model.rectifyImage(depth_img_cv , depth_rect_img_cv )


        # This is stuff for the raw image: 
        raw_pin_model = PinholeCameraModel()
        raw_pin_model.fromCameraInfo(raw_ci)
        raw_blob_marked , raw_keypoints = self.find_donute_process(raw_img_cv)
        cv2.imshow("raw_color", self.scale_image_down(raw_blob_marked , 900))
        cv2.waitKey(1)

        # We ignore all cases where nothing is detected
        if not raw_keypoints:
            return 

        raw_pxy = self.keypoint_to_pxy(self.get_largest_keypoint(raw_keypoints))
        raw_rect_pxy = raw_pin_model.rectifyPoint(raw_pxy)
        raw_rect_pxy = [int(x) for x in raw_rect_pxy]
        raw_rect_depth = depth_rect_img_cv[raw_rect_pxy[1], raw_rect_pxy[0] ]
        raw_rect_rs_sxyz = self.rs_deprojection(depth_ci,raw_rect_pxy, raw_rect_depth  )
        
        # we also compare the pinhole model
        raw_rect_ph_sxyz = [v*raw_rect_depth / 1000  for v in  raw_pin_model.projectPixelTo3dRay(raw_rect_pxy)]
        
        
        print(f"raw rect pixel xy is {raw_rect_pxy[0]:.5f} {raw_rect_pxy[1]:.5f}")
        print(f"raw rect space rs xyz is {raw_rect_rs_sxyz[0]:.5f} {raw_rect_rs_sxyz[1]:.5f} {raw_rect_rs_sxyz[2]:.5f} ")
        print(f"raw rect space ph xyz is {raw_rect_ph_sxyz[0]:.5f} {raw_rect_ph_sxyz[1]:.5f} {raw_rect_ph_sxyz[2]:.5f} ")

        # Do the same process for ract image:
        rect_blob_marked , rect_keypoints = self.find_donute_process(rect_img_cv)
        cv2.imshow("rect_color", self.scale_image_down(rect_blob_marked , 900))
        cv2.waitKey(1)

        # We ignore all cases where nothing is detected
        if not rect_keypoints:
            return 
        rect_pxy = self.keypoint_to_pxy(self.get_largest_keypoint(rect_keypoints))
        rect_depth = depth_rect_img_cv[rect_pxy[1], rect_pxy[0] ]
        rect_sxyz = self.rs_deprojection(depth_ci , rect_pxy , rect_depth)

        print(f"rect pixel xy is {rect_pxy[0]:.5f} {rect_pxy[1]:.5f}")
        print(f"rect space xyz is {rect_sxyz[0]:.5f} {rect_sxyz[1]:.5f} {rect_sxyz[2]:.5f} ")


        color_depth_ts = self.get_transform("D435i_depth_optical_frame","D435i_color_optical_frame" )
        # depth_img.header.frame_id = "D435i_depth_optical_frame"
        self.publish_circle_tf(depth_img.header,"raw_rect_rs" , raw_rect_rs_sxyz)
        self.publish_circle_tf(depth_img.header,"raw_rect_ph" , raw_rect_ph_sxyz)
        self.publish_circle_tf(rect_img.header,"rect_" , rect_sxyz)
        
        color_depth_offset = [color_depth_ts.transform.translation.x,
        color_depth_ts.transform.translation.y,
        color_depth_ts.transform.translation.z,]
        
        
        offset_raw_rect_ph_sxyz = [x1 + x2 for x1,x2 in zip(raw_rect_ph_sxyz , color_depth_offset)]

        self.publish_circle_tf(depth_img.header,"raw_rect_ph_offset" , offset_raw_rect_ph_sxyz)

    def find_donute_process(self , bgr_image:np.ndarray):

        hsv_mask = self.hsv_filter.apply_filter(bgr_image)

        kernel = np.ones((5,5),np.uint8)
        closed_mask = cv2.morphologyEx(hsv_mask, cv2.MORPH_CLOSE, kernel)
        opened_mask = cv2.morphologyEx(closed_mask, cv2.MORPH_OPEN, kernel)
        masked_image = cv2.bitwise_and(bgr_image, bgr_image, mask=opened_mask)

        blob_marked , keypoints = self.blob_detector.apply_filter(masked_image)
        return blob_marked , keypoints

    def get_largest_keypoint(self,keypoints:list):
        max_size = 0
        max_key = None
        for key in keypoints:
            if key.size > max_size:
                max_size = key.size
                max_key = key
        return max_key

    def keypoint_to_pxy(self, keypoint):
        px, py = keypoint.pt
        px= int(px)
        py = int(py)
        return px,py

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
            tf.transform.translation.x = sx
            tf.transform.translation.y = sy
            tf.transform.translation.z = sz
            self.transform_broadcaster.sendTransform(tf)
            i+=1

    def rs_deprojection(self,cam_info:CameraInfo , pxy : list[float,float],depth) -> tuple[float,float,float]:
        intrinsics = self.camera_info_rs_intrinsics(cam_info)
        sx,sy,sz = rs2.rs2_deproject_pixel_to_point(intrinsics, pxy, depth)
        return sx/1000,sy/1000,sz/1000

    def record_camera_info_once(self) ->rs2.intrinsics:
        print(f"Waiting for camera info")
        have_msg , camera_info =wait_for_message(CameraInfo,self, "/D435i/aligned_depth_to_color/camera_info",time_to_wait=10)

        print(f"Got camera info")
        if have_msg is None:
            raise RuntimeError("Did not get camera info after a long wait")
            return
        return self.camera_info_rs_intrinsics(camera_info)

    def publish_circle_tf(self,image_header :Header , child_frame_id :str,  sxyz:list[float]):
        tf = TransformStamped()
        tf.header.stamp = image_header.stamp
        tf.header.frame_id = image_header.frame_id
        tf.child_frame_id = child_frame_id
        tf.transform.translation.x = sxyz[0]
        tf.transform.translation.y = sxyz[1]
        tf.transform.translation.z = sxyz[2]
        print(f"Publishing {tf}")
        self.transform_broadcaster.sendTransform(tf)

    def camera_info_rs_intrinsics(self,cam_info:CameraInfo) ->rs2.intrinsics:
        
        intrinsics = rs2.intrinsics()
        intrinsics.width = cam_info.width
        intrinsics.height = cam_info.height
        intrinsics.ppx = cam_info.k[2]
        intrinsics.ppy = cam_info.k[5]
        intrinsics.fx = cam_info.k[0]
        intrinsics.fy = cam_info.k[4]
        if cam_info.distortion_model == 'plumb_bob':
            intrinsics.model = rs2.distortion.brown_conrady
        elif cam_info.distortion_model == 'equidistant':
            intrinsics.model = rs2.distortion.kannala_brandt4
        intrinsics.coeffs = [i for i in cam_info.d]

        return intrinsics

    def depth_image_cb(self, depth_image):
        self.depth_image_deque.appendleft(depth_image)




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
