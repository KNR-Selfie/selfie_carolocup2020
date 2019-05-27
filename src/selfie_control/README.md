# Selfie control
Consist of nodes:
- cmd_creator
- pid_controller ([link](http://wiki.ros.org/pid))
- offset_combiner
- const_setpoint
## Usage
```
. devel/setup.bash
roslaunch selfie_control control.launch
```
## Topics
### Subscribed topics
- `/position_offset` ([std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html))
- `/heading_offset` ([std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html))
- `/speed` ([std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html))
### Published topics
- `/drive` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/melodic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))

## Services
###
- `/cmd_start_pub`
  - cmd_creator start publishing /drive
- `/cmd_stop_pub`
  - cmd_creator stop publishing /drive

## Parameters
###
- `L` (*float*, default: 0.3)
  - position_offset + L*heading_offset
