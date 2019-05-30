#ifndef STARTING_ACTION_CLIENT_H
#define STARTING_ACTION_CLIENT_H

#include <selfie_msgs/startingAction.h>
#include <selfie_scheduler/client_interface.h>

class StartingProcedureClient : public ClientInterface
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<selfie_msgs::startingAction> ac_;
    selfie_msgs::startingGoal goal_;
public:
    StartingProcedureClient(std::string name);
    ~StartingProcedureClient();

    void setGoal(boost::any goal);
    bool waitForResult(float timeout);
    void cancelAction();

    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const selfie_msgs::startingResultConstPtr& result);
    void activeCb();
    void feedbackCb(const selfie_msgs::startingFeedbackConstPtr& feedback);
    bool getResult();


};

#endif // STARTING_ACTION_CLIENT_H
