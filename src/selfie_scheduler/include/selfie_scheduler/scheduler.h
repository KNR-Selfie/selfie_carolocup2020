#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <selfie_scheduler/scheduler_enums.h>
#include <selfie_scheduler/client_interface.h>
#include <boost/any.hpp>

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

    //params
    int begin_action_;
    float start_distance_;
    float parking_spot_;

    program_state current_car_state_;
    program_state previous_car_state_;
    action current_action_state_;

    std::map<action, ClientInterface*> clients_;
    std::map<action, boost::any> action_args_;
    ClientInterface *current_client_ptr_;

    ros::ServiceClient visionReset_ ;
    ros::ServiceClient cmdCreatorStartPub_;
    ros::ServiceClient cmdCreatorStopPub_;

    template <typename T> bool checkCurrentClientType();
    void stateMachine();
    void startAction(action action_to_set);
    void stopAction();

public:
    Scheduler();
    ~Scheduler();

    void init();

    void loop();

    void actionSetup(); // begin action
    void logFeedback();
    void feedbackStateMachine();

    void resetVision();
    void stopCmdCreator();
    void startCmdCreator();

    action getBeginAction(); //get first action from argument
    void shiftAction();

    void actionStateMachine();
    bool checkIfActionFinished();

};

#endif // SCHEDULER_H