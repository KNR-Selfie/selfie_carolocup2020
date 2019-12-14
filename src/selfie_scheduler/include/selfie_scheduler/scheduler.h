#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <selfie_scheduler/scheduler_enums.h>
#include <selfie_scheduler/client_interface.h>
#include <boost/any.hpp>
#include <std_msgs/UInt8.h>

/*

Params:
- start action + default arguments (pressed button, detected parking spot)
- starting_distance
- parking_spot
-

*/
class Scheduler
{
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // params
    int begin_action_;
    float start_distance_;
    float parking_spot_;
    int parking_steering_mode_;
    int drive_steering_mode_;

    program_state current_car_state_;
    program_state previous_car_state_;
    action current_action_state_;

    std::map<action, ClientInterface*> clients_;
    std::map<action, boost::any> action_args_;
    ClientInterface *current_client_ptr_;

    ros::ServiceClient visionReset_ ;
    ros::ServiceClient cmdCreatorStartPub_;
    ros::ServiceClient cmdCreatorStopPub_;
    ros::ServiceClient avoidingObstSetPassive_;
    ros::ServiceClient avoidingObstSetActive_;
    ros::ServiceClient resetLaneController_;
    ros::ServiceClient steeringModeSetAckermann_;
    ros::ServiceClient steeringModeSetParallel_;
    ros::ServiceClient steeringModeSetFrontAxis_;


    ros::Subscriber switchState_;
    rc_state previousRcState_;
    rc_state currentRcState_;
    void switchStateCallback(const std_msgs::UInt8ConstPtr &msg);

    template <typename T> bool checkCurrentClientType();
    void stateMachine();
    void startAction(action action_to_set);
    void startNextAction();
    void stopAction();

    void setAvoidingObstActive();
    void setAvoidingObstPassive();
    void resetLaneControl();
    void resetVision();
    void stopCmdCreator();
    void startCmdCreator();
    void setParkSteeringMode();
    void setDriveSteeringMode();


public:
    Scheduler();
    ~Scheduler();

    void init();

    void loop();

    void actionSetup(); // begin action
    void logFeedback();
    void feedbackStateMachine();

    void setupActionClients(bool button_pressed);
    void waitForStart();

    action getBeginAction(); //get first action from argument
    void shiftAction();

    void actionStateMachine();
    int checkIfActionFinished();

};

#endif // SCHEDULER_H
