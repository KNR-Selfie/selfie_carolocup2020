#ifndef PARK_ACTION_CLIENT_H
#define PARK_ACTION_CLIENT_H

#include <selfie_msgs/parkAction.h>
#include <selfie_scheduler/client_interface.h>
#include <selfie_scheduler/scheduler_enums.h>

class ParkClient : public ClientInterface
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<selfie_msgs::searchAction> ac_;
    selfie_msgs::searchGoal goal_;
    bool result_;
    program_state action_state_;

public:
    ParkClient(std::string name);
    ~ParkClient();

    void setGoal(boost::any goal);
    bool waitForResult(float timeout);
    void cancelAction();
    bool waitForServer(float timeout);

    program_state getActionState();
    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const selfie_msgs::searchResultConstPtr& result);
    void activeCb();
    void feedbackCb(const selfie_msgs::searchFeedbackConstPtr& feedback);
    bool getResult();


};


#endif // PARK_ACTION_CLIENT_H
