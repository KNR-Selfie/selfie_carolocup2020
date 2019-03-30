# Odometry

`selfie_odometry` package provides a node with the same name responsible for calculating odometry from encoder & IMU data. Results are published both in the `odom` topic and as a [tf2](http://wiki.ros.org/tf2) transformation.

## `selfie_odometry`

## Subscribed topics

`imu` ([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))
Data stream from IMU.

`speed` ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))
Linear velocity magnitude at the center of rear axle, as calculated from encoder data.

## Published topics

`odom` ([nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html))

## Parameters

`~rear_axis_frame` (`string`, default: base_link)
The name of the rear axis frame.

## Provided tf transforms

`odom` → `~rear_axis_frame`
