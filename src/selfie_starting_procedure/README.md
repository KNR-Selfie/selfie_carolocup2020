# Starting procedure
Perfom starting procedure. Wait for button press and run the car to cover given distance.
Package implements action `starting_procedure` triggered by scheduler.

## Usage
```
. devel/setup.bash
rosrun selfie_starting_procedure starting_procedure
```
## Topics
### Subscribed topics

- `/start_button` ([std_msg/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))
  - Pressed one of two task buttons
- `/distance` ([std_msg/Float32](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32.html))
  - Distance covered by the car (based on encoder's reading)

### Published topics
- `/drive` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))
  - Drive commands for uC to run the car
