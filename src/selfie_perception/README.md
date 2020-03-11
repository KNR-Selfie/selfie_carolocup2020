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
- `/intersection_distance` ([std_msgs/Float32](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html))
  - Distance in meters to intersection
- `/starting_line` ([std_msgs/Float32](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html))
  - Distance in meters to staring line

### Used Services
- `/reset_vison` ([std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html))
  - used to reset vision

## Scipts
#### make hom cut file
  - make yaml file containts mask of homography black area cover.

## Parameters
###
- `config_file` (*string*)
  - Path to yaml file with homography matrix
- `hom_cut_file` (*string*)
  - Path to yaml file with mask of homography black area cover
- `debug_mode` (*bool*, default: false)
  - Whether or not opencv visualization windows are displayed
- `real_window_size` (*float*, default: 0.1)
  - Size of adaptive threshold window (m)
- `threshold_c` (*int*, default: -40)
  - Constant subtracted from the mean or weighted mean
- `tune_params_mode` (*bool*, default: false)
  - special mode of the node to tune thresholds parameters
- `max_mid_line_distance` (*float*)
  - max side line distance to merge (used to merge middle lines)
- `max_mid_line_gap` (*float*)
  - max line distance to merge (used to merge middle lines)
- `pf_num_samples` (*int*)
  - number of particles in particle filter
- `pf_num_points_` (*int*)
  - number of control points describing polynomial line in particle filter
- `pf_std_min` (*float*)
  - min standard deviation of normal distribution using to move control points during particle filter perception (linear relationship based on distance of the car)
- `pf_std_max` (*float*)
  - max standard deviation of normal distribution using to move control points during particle filter perception (linear relationship based on distance of the car)
- `pf_num_samples_vis` (*int*)
  - number of particles in particle filter to visualize

# Detect obstacles
![boxes_resized](https://user-images.githubusercontent.com/28540485/48948585-42556d00-ef35-11e8-8e83-6f161eb9e080.png)
## Usage
```
. devel/setup.bash
rosrun selfie_perception detect_obstacles
```
## Topics
### Subscribed topics
- `/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html))
  - Message published by LIDAR
### Published topics
- `/obstacles` (selfie_msgs/msgs/PolygonArray.msg)
  - Polygon Array contains vertexes of every rectangular obstacle in local XY
- `/visualization_lines` ([visualization_msgs/Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))
  - (optional) Visualization of lines
- `/visualization_obstacles` ([visualization_msgs/Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))
  - (optional) Visualization of obstacles
## Parameters
###
- `obstacles_frame` (*string*, default: "laser")
  - The frame attached to the local map
- `visualization_frame` (*string*, default: "laser")
  - The frame attached to the local map visualization
- `max range` (*float*, default: 1.5)
  - Max obstacles detection range (m)
- `min_range` (*float*, default: 0.03)
  - Min obstacles detection range (m)
- `line_min_length` (*float*, default: 0.05)
  - Min legth of line after merge similar lines (m)
- `segment_threshold` (*float*, default: 0.03)
  - Max distance difference between segments (m)
- `min_segment_size` (*float*, default: 0.04)
  - Min size of segment (m)
- `min_to_divide` (*float*, default: 0.03)
  - Min distance to point in segment to divide line (m)
- `lidar_offset` (*float*, default: 0.0)
  - Lidar offset (m)
- `upside_down_` (*bool*, default: false)
  - Whether or not lidar is reversed
- `visualize` (*bool*, default: false)
  - Whether or not visualization markers are published
