# STM32 Bridge

`selfie_stm32_bridge` package provides a node with the same name responsible for communication with on-board STM32 microcontroller that handles IMU, encoders and vehicle control. The node communicates with ROS using standarized message types, as described below.

## `selfie_stm32_bridge`

## Subscribed topics

`drive` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))
Steering commands to be applied.

`left_turn_indicator` ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))
Status of left turn indicator.

`right_turn_indicator` ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))
Status of right turn indicator.

## Published topics

`imu` ([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))
Data stream from IMU.

`speed` ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))
Linear velocity magnitude at the center of rear axle, as calculated from encoder data (in m/s).

`start_button1` ([std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html))
starts parking competition

`start_button2` ([std_msgs/Empty](http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html))
starts obstacle competition

`distance` ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))
Distance covered by the car.

`switch_state` ([std_msgs/UInt8](http://docs.ros.org/api/std_msgs/html/msg/UInt8.html))
Car drive mode set by user on RC.
2 - manual, 1- semi-autonomous 0 - autonomous mode

## Published services

`ackerman_steering_mode` ([std_srv/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
Service used to set ackerman steering mode (front and back axis move reversely)

`parallel_steering_mode` ([std_srv/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
Service used to set parallel steering mode (front and back axis move in the same direction)

`front_axis_steering_mode` ([std_srv/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
Service used to set front axis steering mode (only front axis moves)


## Parameters
Below parameters are used to compensate mechanical shortcomings of servomechanisms

`~ackermann_offset_front`(`float`)
Offset for front steering axle in ackerman steering mode 

`~ackermann_offset_back`(`float`)
Offset for back steering axle in ackerman steering mode 

`~parallel_offset_front_right`(`float`)
Offset for front steering axle (turning rigt) in parallel steering mode 

`~parallel_offset_back_right`(`float`)
Offset for back steering axle (turning rigt) in parallel steering mode 

`~parallel_offset_front_left`(`float`)
Offset for front steering axle (turning left) in ackerman steering mode 

`~parallel_offset_back_left`(`float`)
Offset for back steering axle (turning left) in ackerman steering mode 

`~front_axis_offset`(`float`)
Offset for front steering axle in front-axis steering mode 

`~back_axis_offset`(`float`,default:)
Offset for back steering axle in front-axis steering mode 



