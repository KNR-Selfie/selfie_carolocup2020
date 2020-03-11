# Free drive Action

Free drive action server- waiting an event to occur while free ride. When parking scenario server controls speed.

Server uses [driving.action](https://github.com/KNR-Selfie/selfie_carolocup2020/wiki/Messages-and-actions) to communicate with client

## Usage

```

source ./devel/setup.bash

rosrun selfie_free_drive intersection_free_drive

```

## Topics
### Action name
- `free_drive`


### Subscribed topics

-  `/distance` ([std_msgs/Float32](https://docs.ros.org/api/std_msgs/html/msg/Float32.html))
   - detected obstacles

-  `/intersection_distance` ([std_msgs/Float64](https://docs.ros.org/api/std_msgs/html/msg/Float64.html))
   - distance to the nearest intersection

-  `/starting_line` ([std_msgs/Float64](https://docs.ros.org/api/std_msgs/html/msg/Float64.html))
   - distance to the nearest starting line

### Published topics

-  `/max_speed` ([std_msgs/Float64](https://docs.ros.org/api/std_msgs/html/msg/Float64.html))
   - current speed of car

  
  

## Parameters

-  `max_speed` (*float*)
   - max speed of the car when parking scenario
-  `starting_line_distance_to_end` (*float*)
   - distance to starting line causing the action server to end
-  `intersection_distance_to_end` (*float*)
   - distance to intersection causing the action server to end
-  `distance_to_verify_event` (*float*)
   - distance covered after event required to pass next event (preventing inf loop on intersection)