#ifndef SEARCH_ACTION_CLIENT_H
#define SEARCH_ACTION_CLIENT_H

#include <selfie_msgs/searchAction.h>
#include <selfie_scheduler/client_interface.h>
#include <selfie_scheduler/scheduler_enums.h>

class SearchClient : public ClientInterface
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<selfie_msgs::searchAction> ac_;
    selfie_msgs::searchGoal goal_;
    geometry_msgs::Polygon result_;
    program_state action_state_;
    bool result_flag_;
    action next_action_;

public:
    SearchClient(std::string name);
    ~SearchClient();

    void setGoal(boost::any goal);
    bool waitForResult(float timeout);
    void cancelAction();
    bool waitForServer(float timeout);

    program_state getActionState();
    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const selfie_msgs::searchResultConstPtr& result);
    void activeCb();
    void feedbackCb(const selfie_msgs::searchFeedbackConstPtr& feedback);
    void getActionResult(boost::any &result);
    bool isActionFinished();
    action getNextAction();
};

#endif // SEARCH_ACTION_CLIENT_H
