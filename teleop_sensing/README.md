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

This is only working-ish. 

Launch the file directly: 
```
teleop_sensing/cv_process.py
```

This will open an open-cv window with trackbar to tune the filter. 

When using this, the HSV values are likely needs to change for the center of the ring being detected. 

This node will publish a TF (with wrong orientation) of the center of the ring, located at the btm surface of the ring. 

The TF will be named like `keypoint_{i}`


