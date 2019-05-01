#ifndef SCHEDULER_ENUMS_H
#define SCHEDULER_ENUMS_H
typedef enum program_states
{
    //starting_procedure
    SELFIE_READY = 0, // car ready, waiting for button press
    START_SIGN, // button pressed
    START_DRIVE, // car started to drive
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

#endif // SCHEDULER_ENUMS_H