# Selfie scheduler
Package provides serveral action clients and schdduler node.

## `selfie_scheduler`
Package used to administer switching actions (scenarios).
![scheduler_flow_chart](https://user-images.githubusercontent.com/26739110/75875922-f28e7280-5e14-11ea-848b-db97cc750ab8.PNG)

## Action clients
Action clients enable starting and stopping action servers placed in other packages.
Common methods for action clients have been implemented as interface class in file `client_interface.h`.
In current implementation following clients are used:
* starting_action_client
* drive_action_client
* search_ation_client
* park_action_client
* intersection_action_client

### Subscribed topics
`switch_state` ([std_msgs/Uint8](http://docs.ros.org/kinetic/api/std_msgs/html/msg/UInt8.html))
- state of RC mode (0 - manual, 1 - half-autonomous, 2 - autonomous)


## Parameters

`~begin_action` (*int*, default: 1)
Indicates which action car should start at first. Possible values:
  - 1 - STARTING_PROCEDURE
  - 2 - FREE_DRIVE
  - 3 - PARKING_SEARCH
  - 4 - PARKING

`~starting_distance` (*float*, default: 1.0)
Distance car should cover to drive out of the starting box (m)

`~parking_spot` (*float*, default: 0.5)
Minimum parking spot width (m)

`~parking_steering_mode`(*int*, default: 0)
Steering mode during parking phase(0 - ackermann 1 - paralell)

`~drive_steering_mode`(*int*, default 0)
Steering mode during drive (0 - ackermann  1 - paralell 2 - front axis only) 



