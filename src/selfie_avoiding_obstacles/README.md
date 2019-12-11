# Avoiding Obstacles 
lane_controller- in active mode detects obstacles on the road, changes lane and returns after overtaking. In passive mode sends constant setpoint.
Current version overtakes static obstacles.


## Usage
```
. devel/setup.bash
rosrun selfie_avoiding_obstacles lane_controller
```
## Topics
### Subscribed topics
- `/obstacles` ([selfie_msgs/PolygonArray](https://github.com/KNR-Selfie/selfie_carolocup2020/wiki/Messages-and-actions))
  - detected obstacles
 - `/road_markings` ([selfie_msgs/RoadMarkings](https://github.com/KNR-Selfie/selfie_carolocup2020/wiki/Messages-and-actions))
   - contains polynomial coefficients for fitted lines
 
### Published topics
- `/setpoint` ([std_msgs/Float32](https://docs.ros.org/api/std_msgs/html/msg/Float32.html))
  - describes lane, changing this value changes lane
- `/max_speed` ([std_msgs/Float64](https://docs.ros.org/api/std_msgs/html/msg/Float64.html))
  - current speed of car
- `/avoiding_obstacles` ([visualization_msgs/Marker](https://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))
  - (only when parameter `visualization=true` visualizes found found and places in rviz)

### Used Services
- `/avoiding_obst_set_passive` ([std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html))
- `/avoiding_obst_set_passive` ([std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html))
  - they are used to switch modes on active and passive
- `/resetLaneControl` ([std_srvs/Empty](https://docs.ros.org/api/std_srvs/html/srv/Empty.html))
  - used to reset node


## Parameters
 - `ROI_min_x`,`ROI_min_y`,`ROI_max_x`,`ROI_max_x` (*float*)
   - describing area of interest
 - `maximum_length_of_obstacle` (*float*, default: 0.8)
   - How long can be approached obstacle (described in regulations)
 - `right_lane_setpoint`,`left_lane_setpoint` (*float*, default: -0.2,0.2)
   - How far from middle of road is middle of right and left lane
 - `maximum_speed` (*float*)
 - `safe_speed` (*float*)
   - Speed used when lane is being changed
 - `safety_margin` (*float*)
  - safety margin considering inaccurations in measuring distance, used to calculate 
 - `visualization` (*bool*, default: true)
   - Whether or not visualization topics are active
