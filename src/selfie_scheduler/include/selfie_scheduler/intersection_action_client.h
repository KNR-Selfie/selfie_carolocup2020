#ifndef INTERSECTION_ACTION_CLIENT_H
#define INTERSECTION_ACTION_CLIENT_H
#include <selfie_msgs/intersectionAction.h>
#include <selfie_scheduler/client_interface.h>
#include <selfie_scheduler/scheduler_enums.h>

class IntersectionClient : public ClientInterface
{
protected:
    actionlib::SimpleActionClient<selfie_msgs::intersectionAction> ac_;
    selfie_msgs::intersectionGoal goal_;
    ros::ServiceClient avoidingObstSetPassive_;
    ros::ServiceClient resetLaneController_;
    bool result_;
public:
    IntersectionClient(std::string name);
    ~IntersectionClient();

    void setGoal(boost::any goal);
    bool waitForResult(float timeout);
    void cancelAction();
    bool waitForServer(float timeout);

    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const selfie_msgs::intersectionResultConstPtr& result);
    void activeCb();
    void feedbackCb(const selfie_msgs::intersectionFeedbackConstPtr& feedback);
    bool getResult();
    void getActionResult(boost::any &result);
    void prepareAction();
};

#endif // INTERSECTION_ACTION_CLIENT_H
