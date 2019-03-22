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
- `line_search_max_range_difference` (*float*, default: 0.04)
  - Max distance between two points in same line (m)
- `line_search_max_slope_difference` (*float*, default: 2.0)
  - Max dynamic slope difference between slope of starting point and actual point and average of previous slopes in same line (rad)
- `line_search_min_slope_difference` (*float*, default: 0.05)
  - Min dynamic slope difference between slope of starting point and actual point and average of previous slopes in same line (rad)
- `line_search_slope_difference_ratio` (*float*, default: 0.06)
  - Ratio in dynamic slope difference. Lower value means faster change
- `line_search_min_length` (*float*, default: 0.014)
  - Min length of line before merge similar lines (m)
- `line_min_length` (*float*, default: 0.05)
  - Min legth of line after merge similar lines (m)
- `obstacle_nominal_length` (*float*, default: 0.11)
  - Obstacle length when detected only one line (m)
- `visualize` (*bool*, default: true)
  - Whether or not visualization markers are published
