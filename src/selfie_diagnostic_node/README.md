# Diagnostics

`selfie_diagnostic_node` package provides a node responsible for checking sensors connection at the start of robot system. Status for all sensros is published to the `diagnose/selfie_diagnostics` topic. Moreover the status of robot is shown through leds blinking.

## `selfie_diagnostic_node`

## Subscribed topics

Subscribed topics are configurable and can be changed in `parameters.yaml` file, which is placed in `selfie_launch` package.

For this configuration the following topics are being subscribed:


`imu` ([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))
Data stream from IMU.

`stm32/speed` ([std_msgs/Float32](http://docs.ros.org/api/std_msgs/html/msg/Float32.html))
Linear velocity magnitude at the center of rear axle, as calculated from encoder data.

`image_rect` ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
Uncompressed image from camera placed on robot.

`scan` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))
Single scan from a planar laser range-finder

## Published topics

Published topics are configurable and can be changed in `parameters.yaml` file, which is placed in `selfie_launch` package.

For this configuration the following topics are being published.

`~selfie_diagnostics` ([std_msgs/String](http://docs.ros.org/melodic/api/std_msgs/html/msg/String.html))
Complex information about every sensor.

Additionally every sensor publishes it's own message to topic given by template


`~<name>` ([std_msgs/String](http://docs.ros.org/melodic/api/std_msgs/html/msg/String.html))
Complex information single sensor. `<name>` is provided from parameters.

## Parameters

This node searches for 5 parameters for each sensor. Template looks like following:


`<number>_sensor_datatype` (`string`)
Ros message type incoming from topic.


`<number>_sensor_directory` (`string`) 
When this parameter is not set, then algorithm doesn't check if sensor is plugged in.


`<number>_sensor_hz` (`number`)
Frequency of sensor.


`<number>_sensor_name` (`string`)
Name being published in topics messages.


`<number>_sensor_topic` (`string`)
Topic where the data form sensor comes.