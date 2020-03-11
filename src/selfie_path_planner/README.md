# Selfie path planner
Calculate position and heading offsets between car and lane.

Speed control based on road curvature.
## Usage
```
. devel/setup.bash
rosrun selfie_path_planner extract_road_features
```
## Topics
### Subscribed topics
- `/road_markings` (selfie_msgs/RoadMarkings)
- `/max_speed` ([std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html))
### Published topics
- `/position_offset` ([std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html))
- `/heading_offset` ([std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html))
- `/speed` ([std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html)

## Parameters
###
- `lookahead` (*float*, default: 0.0)
  - Distance from the car where offsets are calculated [m]
- `min_spped` (*float*, default: 0.5)
  - Min speed of the car [m/s]
-  `max_acceleration` (*float*)
   - max acceleration of the car
-  `max_deceleration` (*float*)
   - max deceleration of the car
-  `max_curvature` (*float*)
   - max possible curvature (linear speed control)
-  `average_window_size` (*float*)
   - window size in moving average filter that smooth curvature of the road