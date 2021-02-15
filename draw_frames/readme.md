This package plots a circle on the projection of a tf onto the camera image

Instructions:

Assumming you have a bag with these topics: 

/HX09/camera/color/image_raw/compressed
/HX09/camera/color/camera_info
/tf


then, you have to do

```
rosbag play NAME.bag --clock
roslaunch draw_frames draw_frames.launch 
```
