# Selfie scheduler
Package provides serveral action clients and schdduler node.

## Scheduler
Node used to administer switching actions (scenarios).

### Usage
```
. devel/setup.bash
rosrun selfie_scheduler scheduler
```

### Parameters
###
- `begin_action` (*int*, default: 1)
  - Indicates which action car should start at first. Possible values:
    - 1 - STARTING_PROCEDURE
    - 2 - FREE_DRIVE
    - 3 - PARKING_SEARCH
    - 4 - PARKING
- `starting_distance` (*float*, default: 1.0)
  - Distance car should cover to drive out of the starting box (m)
- `parking_spot` (*float*, default: 0.5)
  - Minimum parking spot width (m)


### Services
### Clients
- `/resetVision` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
  - Published to service responsible for reseting vision system
- `/cmd_start_pub` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
  - Published to service resposible for starting cmd creator
- `/cmd_stop_pub` ([std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html))
  - Published to service resposible for stopping cmd creator

## Action clients
Action clients enable starting and stopping action servers placed in other packages.
Common methods for action clients have been implemented as interface in file `client_interface.h`.
Following clients are used:
* starting_action_client
* drive_action_client
* search_ation_client
* park_action_client

