# Detect markings
![markings_pic](https://user-images.githubusercontent.com/28540485/54884320-bb2aee00-4e6f-11e9-8b67-3f326029e4e9.png)
## Usage
```
. devel/setup.bash
rosrun selfie_perception detect_markings
```
## Topics
### Subscribed topics
- `/image_rect` ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
  - Image from camera
### Published topics
- `/road_markings` (selfie_msgs/msgs/RoadMarkings.msg)
  - Polynomial coefficients for fitted lines, starting from the constant term
- `/intersection` ([std_msgs/Float32](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html))
  - Distance in meters to intersection
- `/starting_line` ([std_msgs/Float32](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html))
  - Distance in meters to staring line
## Parameters
###
- `config_file` (*string*)
  - Path to yaml file with homography matrix
- `debug_mode` (*bool*, default: false)
  - Whether or not opencv visualization windows are displayed
- `real_window_size` (*float*, default: 0.1)
  - Size of adaptive threshold window (m)
- `threshold_c` (*int*, default: -40)
  - Constant subtracted from the mean or weighted mean
