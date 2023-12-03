# Visual Sensing

This package is to find the 5 rings within camera's field of view, publish their transform. As well as detecting apriltags and their transforms.
find rings, april-tags, and publish their tfs.

## Hardware requirement

Intel Realsense D435(i) connected to the machine running this package.

## Running this package

Everything will be launched by running the launch file 
```
ros2 launch teleop_sensing rings_cv.launch.xml
```

To see the cv2 window with trackbars to live tune the filer parameters, set the debug argument to true 
```
ros2 launch teleop_sensing rings_cv.launch.xml debug:=True
```

[example of running this in debug mode](https://github.com/ME495-EmbeddedSystems/final-project-teleop/assets/7969697/c79e4f70-6d18-47e0-8438-9ad4425930bc)

### April tag tf names 

The actual configuration for each tag is in the `tag16_config.yaml`

they would be 

* tag16H05_0 - not assigned
* tag16H05_1 - not assigned 
* tag16H05_2 - Workspace-table
* tag16H05_3 - ABB table
* tag16H05_10 - peg


For each ring detected, a transform will be published at it's center

**Node: The published TF will go through the middle of the ring and actually land on the background object. This works fine when ring is on the table, the Frame will simply be sitting on the table**

Transforms will be named as `<COLOR>_center`

**All transforms published will be based on the D435i's base frame**


## More detailed workflow explanation

### April Tag detection

The following parts are working together for april-tag detection:
* realsense_ros: publish camera images
* image_proc: rectify image
* apriltag_ros: finding april-tag and publishing tag's TF

There is a special launch file `image_proc_with_remap.launch.py` to launch the image_proc with topic re-mapping. ROS2 removes the ability to remap topics on launch file include. Thus this is needed to have image_proc subscribe the correct image topic.

The `apriltag_ros` handles the actual detection of the apriltag. A config file `tag16_config.yaml` list the info for each apriltag used in the project.

### Ring Locating

The workflow for ring detection use the following nodes
* realsense_ros: publish alighed color and depth image
* cv_process: python node to detect ring and locate it.

The detection process is generally a classic CV problem. 
* HSV filter 
* Blob-detection

After ring is located inside a image frame. It's location in space is found through 
* rectifying
* de-projection

Finally, the ring's location is published as a TF

