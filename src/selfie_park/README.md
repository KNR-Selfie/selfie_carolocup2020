# Search Action
Search action server- manages speed of car while searching for free place in parking zone
Server uses [search.action](https://github.com/KNR-Selfie/selfie_carolocup2020/wiki/Messages-and-actions) to communicate with client
## Usage
```
. devel/setup.bash
rosrun selfie_park detect_parking_spot
```
## Topics
###Action name
- `search`

### Subscribed topics
- `/obstacles` ([selfie_msgs/PolygonArray](https://github.com/KNR-Selfie/selfie_carolocup2020/wiki/Messages-and-actions))
  - detected obstacles
 
### Published topics
- `/max_speed` ([std_msgs/Float64](https://docs.ros.org/api/std_msgs/html/msg/Float64.html))
  - current speed of car
- `/free_place` ([visualization_msgs/Marker](https://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))
  - (only when parameter `visualization=true` visualizes found boxes and places in rviz)


## Parameters
###
 - `point_min_x`,`point_min_y`,`point_max_x`,`point_max_x` (*float*)
   - describing area of interest
 - `visualization_in_searching` (*bool*, default: true)
   - Whether or not visualization topics are active
 - `default_speed_in_parking_zone` (*float*, default: 0.8)
 - `speed_when_found_place` (*float*, default: 0.3)
   - speed when found possible free place
 - `box_angle_deg` (*float*, default: 55)
   - describes maximum angle (in degrees) between car and found place (used mainly in filtering out wrong places)
 - `max_distance_to_free_place` (*float*, default: 0.8)
   - describes maximum angle between car and found place (used mainly in filtering out wrong places)
 - `length_of_parking_area` (*float*, default: 0.8)
   - how long will be covered before cancelling searching


# Park Action
## Usage
```
. devel/setup.bash
rosrun selfie_park park_server
```
## Topics
### Subscribed
- /distance (std_msgs/Float32)
### Published
- /drive (ackermann_msgs/AckermannDriveStamped)
- /right_turn_indicator (std_msgs/Bool)
- /left_turn_indicator (std_msgs/Bool)

## Parameters
- state_msgs (bool)
printing messages indicating the current state of the parking manouvre
- parking_speed (float)
distance between imu and the laser sensor of the vehicle
- back_to_mid (float)
distance between the back and the base_link of the vehicle
- idle_time (float)
time spent idle in the parking spot
- iter_distance (float)
one move distance
- angle_coeff (float)
angle coefficient for calculations
- max_turn (float)
maximal wheel turn angle
- turn_delay (float)
time to wait for turn direction change
