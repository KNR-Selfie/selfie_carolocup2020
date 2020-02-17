#ifndef CLIENT_INTERFACE_H
#define CLIENT_INTERFACE_H

#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <selfie_scheduler/scheduler_enums.h>
#include <boost/any.hpp>

class ClientInterface
{
protected:
    ros::NodeHandle nh_;
    program_state action_state_;
    client_goal_state result_flag_;
    action next_action_;
    
public:
    virtual ~ClientInterface() = 0;
    virtual bool waitForServer(float timeout) = 0;
    virtual void setGoal(boost::any goal) = 0;
    virtual bool waitForResult(float timeout) = 0;
    virtual void cancelAction() = 0;
    virtual void getActionResult(boost::any &result) = 0;
    virtual void prepareAction() = 0;

    client_goal_state getClientGoalState() {return result_flag_;}
    action getNextAction(){return next_action_;}
    program_state getActionState()
    {
        if(action_state_ != SELFIE_IDLE)
            return action_state_;
    }

};
#endif // CLIENT_INTERFACE_H
