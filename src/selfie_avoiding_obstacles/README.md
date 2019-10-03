# Avoiding Obstacles Action
detect_road_obstacle- detects obstacles on the road, changes lane and returns after overtaking.
Current version overtakes static obstacles.


## Usage
```
. devel/setup.bash
rosrun selfie_avoiding_obstacles detect_road_obstacle
```
## Topics
### Subscribed topics
- `/obstacles` ([selfie_msgs/PolygonArray](https://github.com/KNR-Selfie/selfie_carolocup2020/wiki/Messages-and-actions))
  - detected obstacles
 - `/road_markings` ([selfie_msgs/RoadMarkings](https://github.com/KNR-Selfie/selfie_carolocup2020/wiki/Messages-and-actions))
   - contains polynomial coefficients for fitted lines
 
### Published topics
- `/max_speed` ([std_msgs/Float64](https://docs.ros.org/api/std_msgs/html/msg/Float64.html))
  - current speed of car
- `/avoiding_obstacles` ([visualization_msgs/Marker](https://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))
  - (only when parameter `visualization=true` visualizes found found and places in rviz)


## Parameters
###
 - `visualization` (*bool*, default: false)
   - Whether or not visualization topics are active
