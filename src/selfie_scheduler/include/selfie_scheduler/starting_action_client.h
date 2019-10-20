#ifndef STARTING_ACTION_CLIENT_H
#define STARTING_ACTION_CLIENT_H

#include <selfie_msgs/startingAction.h>
#include <selfie_scheduler/client_interface.h>
#include <selfie_scheduler/scheduler_enums.h>
class StartingProcedureClient : public ClientInterface
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<selfie_msgs::startingAction> ac_;
    selfie_msgs::startingGoal goal_;
    bool result_;
    program_state action_state_;
    int result_flag_;
    action next_action_;

public:
    StartingProcedureClient(std::string name);
    ~StartingProcedureClient();

    void setGoal(boost::any goal);
    bool waitForResult(float timeout);
    void cancelAction();
    bool waitForServer(float timeout);

    program_state getActionState();
    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const selfie_msgs::startingResultConstPtr& result);
    void activeCb();
    void feedbackCb(const selfie_msgs::startingFeedbackConstPtr& feedback);
    int isActionFinished();
    void getActionResult(boost::any &result);
    action getNextAction();


};

#endif // STARTING_ACTION_CLIENT_H
