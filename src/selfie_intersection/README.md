# Intersection Action

Intersection action server- Controls speed of car when approached intersection

Server uses [intersection.action](https://github.com/KNR-Selfie/selfie_carolocup2020/wiki/Messages-and-actions) to communicate with client

## Usage

```

source ./devel/setup.bash

rosrun selfie_intersection intersection_server

```

## Topics

### Subscribed topics

-  `/obstacles` ([selfie_msgs/PolygonArray](https://github.com/KNR-Selfie/selfie_carolocup2020/wiki/Messages-and-actions))

   - detected obstacles

### Published topics

-  `/max_speed` ([std_msgs/Float64](https://docs.ros.org/api/std_msgs/html/msg/Float64.html))

   - current speed of car

-  `/intersection` ([visualization_msgs/Marker](https://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))

   - (only when parameter `visualization=true` visualizes found obstacles)

  
  

## Parameters

###

-  `point_min_x`,`point_min_y`,`point_max_x`,`point_max_x` (*float*)

   - describing area of interest

-  `visualization` (*bool*, default: false)

   - Whether or not visualization topics are active
