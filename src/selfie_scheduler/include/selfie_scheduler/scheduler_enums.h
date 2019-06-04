#ifndef SCHEDULER_ENUMS_H
#define SCHEDULER_ENUMS_H
typedef enum program_state
{
    //starting_procedure
    SELFIE_IDLE = 0,
    SELFIE_READY, // car ready, waiting for button press
    BUTTON_FREE_DRIVE_PRESSED, // button starting Free Drive and Parking
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
    FIND_PLACE, // parking place found (don't know if suitable)
    FIND_PROPER_PLACE, // suitable parking place found
    START_PARK, // parking procedure started
    IN_PLACE, // car parked
    OUT_PLACE, // car drives out of parking place
    READY_TO_DRIVE, // car ready to further ride

}feedback_variable;


typedef enum action
{
    IDLE = 0,
    STARTING,
    DRIVING,
    PARKING,
    PARK,
    ERROR,

}action_variable;
#endif // SCHEDULER_ENUMS_H
