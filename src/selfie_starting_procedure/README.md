# Starting procedure
Perfom starting procedure. Wait for button press and start searching for the gate and qr code. Starts When either of them is found or a button is pressed. The last buttonpress decides the competition.
Package implements action `starting_procedure` triggered by scheduler.

## Usage
```
rosrun selfie_starting_procedure starting_procedure
```
## Topics
### Subscribed topics
- `/start_button1` ([std_msg/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html))
   indicates the parking competition button was pressed
- `/start_button2` ([std_msg/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html))
   indicates the obstacles competition button was pressed
- `/distance` ([std_msg/Float32](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html))
   Distance covered by the car (based on encoder's reading)
- `/odom` ([nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html))
   feedback to the proportional regulator
### Published topics
- `/drive` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))
   Drive commands for uC to run the car

## Called services
- `/startQrSearch` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
   Called when a button is pressed, starts searching for the qr code
- `/stopQrSearch` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
   Called when a button is pressed again, indicates the search os no longer necessary
- `/startGateScan` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
   Called when a button is pressed, starts searching for the gate with the lidar sensor

## Parameters
- `starting_speed` (float, default=2)
- `use_scan` (bool, default=false)
   whether to start when the lidar stops detecting the gate
- `use_qr` (bool, default=true)
   whether to start when the camera stops detecting the qr code
- `Kp` (float, default=1)
   the proportional gain of starting regulator, keeps the direction throughout the starting procedure

# Qr Decoder
## Usage
```
rosrun selfie_starting_procedure qr_decoder
```

## Topics
### Published topics
- `/qr_gate_open` ([std_msg/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html))
   indicates the starting gate was opened
### Subscibed topics
- `/image_rect` ([sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
   used to detect the qr code

## Advertised services
- `startQrSearch` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
   Starts the search for the qr code
- `stopQrSearch` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
   Stops the search for the qr code
## Parameters
- `min_detect_rate` (float, default=0.4)
   the rate of detection that needs to be exceeded 
- `iterations_to_valid` (int, default=2)
   minimal number of timer iterations with detect rate over the threshold in order to validate the detection of the qr code
- `visualize` (bool, default=false)
   open the visualization of the qr code detection

# Gate scanner
## Usage
```
rosrun selfie_starting_procedure gate_scanner
```

## Topics
### Published topics
- `/scan_gate_open` ([std_msg/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html))
   indicates the starting gate was opened
### Subscibed topics
- `/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))
   used to detect the gate

## Advertised services
- `startGateScan` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
   starts searching for the gate with the lidar sensor when called

## Parameters
- `no_obstacle_time_thresh` (float, default=1.5)
   when there are no detections throughout this amount of time, the feedback is sent
- `min_distance` (float, default=0.1)
   start distance of ROI
- `max_distance` (float, default=0.5)
   end distance of ROI
- `min_width` (float, default=0.2)
   width of ROI
- `min_gate_seen_count` (float, default=5)
   minimal amount of detections to send the feedback
