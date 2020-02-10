#ifndef SCHEDULER_ENUMS_H
#define SCHEDULER_ENUMS_H

//#include <map>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleClientGoalState::StateEnum State;
typedef enum program_state
{
    //generic states
    SELFIE_IDLE = 0,
    SELFIE_READY, // car ready, waiting for button press
    SELFIE_NO_ACTION,

    //starting_procedure
    BUTTON_PARKING_DRIVE_PRESSED, // button starting Free Drive and Parking
    BUTTON_OBSTACLE_DRIVE_PRESSED, // button starting Obstacle Evasion Course
    START_DRIVE, // car started to move
    END_DRIVE, // car drove given distance

    //track ride
    AUTONOMOUS_DRIVE, // autonomous ride
    DETECT_START_LINE, // start line detected
    DETECT_CROSS_LINE, // cross line detected
    DETECT_CROSSROAD, // intersection detected
    DETECT_OBSTACLE, // obstacle detected
    CHANGE_LANE, // change lane

    //parking
    START_SEARCHING_PLACE, //start to search parking place
    FOUND_PLACE_MEASURING, // parking place found, now car is stopped and measures it again
    FIND_PROPER_PLACE, // suitable parking place found
    START_PARK, // parking procedure started
    IN_PLACE, // car parked
    OUT_PLACE, // car drives out of parking place
    READY_TO_DRIVE, // car ready to further ride

    //intersection
    APPROACHING_TO_INTERSECTION, //Approaching to intersection
    APPROACHING_TO_INTERSECTION2, //Approaching to intersection when we can't see line //TODO change it later
    STOPPED_ON_INTERSECTION,  //car stopped before intersection, because there is another car on priority road
    WAITING_ON_INTERSECTION, //car stops on intersection even if there aren't any obstacles and waits certain amount of time
    ROAD_CLEAR  //none obstacles on priority road, action ends now


}feedback_variable;

//std::map<program_state,std::string> program_state_string
//{
//    {SELFIE_IDLE,"SELFIE_IDLE"},
//    {SELFIE_READY,"SELFIE_READY"}

//};
typedef enum action
{
    IDLE = 0,
    STARTING,
    DRIVING,
    PARKING_SEARCH,
    PARK,
    INTERSECTION,
    ERROR,

}action_variable;

typedef enum rc_state
{
    RC_UNINTIALIZED = -1,
    RC_MANUAL = 0,
    RC_HALF_AUTONOMOUS,
    RC_AUTONOMOUS,

}rc_state_variable;

typedef enum client_goal_state
{
    EMPTY,
    ABORTED,
    SUCCESS,

}client_goal_state_variable;

typedef enum steering_mode
{
    ACKERMANN = 0,
    PARALLEL,
    FRONT_AXIS
}steering_mode_variable;

typedef enum park_counter
{
    PARKING_COMPLETE = 5,

}park_counter_variable;
#endif // SCHEDULER_ENUMS_H
