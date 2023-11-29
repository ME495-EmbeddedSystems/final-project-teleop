## Sensing 

find rings, april-tags, and publish their tfs.

## Hardware requirement

Intel Realsense D435(i)

## Run april tag detection 

```
ros2 launch teleop_sensing launch_RS_April.xml
```

This launch 
* realsense_ros: publish camera images
* image_proc: rectify image
* apriltag_ros: finding april-tag and publishing tag's TF


### April tag tf names 

The actual configuration for each tag is in the `tag16_config.yaml`

they would be 

* tag16H05_0 - not assigned
* tag16H05_1 - not assigned 
* tag16H05_2 - Workspace-table
* tag16H05_3 - ABB table
* tag16H05_10 - peg


## Run the object finding

The cv_process will do the classic CV to find the location of rings, and using depth image to deproject a world location of each ring. 

For each ring (color) The following are used: 

* HSV filter
* Blob-detection
* rectifying
* de-projection
* publish tf. 

For simplicity and performance, the S and V channel in HSV are pre-processed on the image before splitting into each color channel. Thus the S and V value among file different rings are shared. 

run `cv_process.py` to start the processing node. This node listen to `/D435i/color/image_row` and `/D435i/aligned_depth_to_color/image_raw` and their camera_info. After launch, it will create 7 cv2 windows. 5 for each color, which also have trackbar for Hue channel of this color. 1 for shared S and V trackbar, 1 for Blob-detector parameter. Live tuning the CV filter is possible through these trackbars. 

For each ring detected, a transform will be published at it's center

**Node: The published TF will go through the middle of the ring and actually land on the background object. This works fine when ring is on the table, the Frame will simply be sitting on the table**

Transforms will be named as `<COLOR>_center`

An additional launch file is created 

```
ros2 launch teleop_sensing ring_cv.launch.xml
```

To not show the cv2 window that allows live tuning of filter parameters: 
```
ros2 launch ring_cv.launch.xml debug:=False
```
