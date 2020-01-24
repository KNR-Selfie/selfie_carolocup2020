#ifndef STARTING_ACTION_CLIENT_H
#define STARTING_ACTION_CLIENT_H

#include <selfie_msgs/startingAction.h>
#include <selfie_scheduler/client_interface.h>
#include <selfie_scheduler/scheduler_enums.h>
class StartingProcedureClient : public ClientInterface
{
protected:
    actionlib::SimpleActionClient<selfie_msgs::startingAction> ac_;
    selfie_msgs::startingGoal goal_;
    bool result_;

public:
    StartingProcedureClient(std::string name);
    ~StartingProcedureClient();

    void setGoal(boost::any goal);
    bool waitForResult(float timeout);
    void cancelAction();
    bool waitForServer(float timeout);

    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const selfie_msgs::startingResultConstPtr& result);
    void activeCb();
    void feedbackCb(const selfie_msgs::startingFeedbackConstPtr& feedback);
    void getActionResult(boost::any &result);
    void prepareAction();
};

#endif // STARTING_ACTION_CLIENT_H
