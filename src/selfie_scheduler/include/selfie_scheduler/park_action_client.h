#ifndef PARK_ACTION_CLIENT_H
#define PARK_ACTION_CLIENT_H

#include <selfie_msgs/parkAction.h>
#include <selfie_scheduler/client_interface.h>
#include <selfie_scheduler/scheduler_enums.h>

class ParkClient : public ClientInterface
{
protected:
    actionlib::SimpleActionClient<selfie_msgs::parkAction> ac_;
    selfie_msgs::parkGoal goal_;
    int result_;
    ros::NodeHandle pnh_;
    int parking_steering_mode_;
    ros::ServiceClient cmdCreatorStopPub_;
    ros::ServiceClient steeringModeSetAckermann_;
    ros::ServiceClient steeringModeSetParallel_;

    int sucessful_park_counter_;
    int park_atttempts_counter_;

public:
    ParkClient(std::string name, const ros::NodeHandle &pnh);
    ~ParkClient();

    void setGoal(boost::any goal);
    bool waitForResult(float timeout);
    void cancelAction();
    bool waitForServer(float timeout);

    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const selfie_msgs::parkResultConstPtr& result);
    void activeCb();
    void feedbackCb(const selfie_msgs::parkFeedbackConstPtr& feedback);
    bool getResult();
    void getActionResult(boost::any &result);
    void setParkSteeringMode();
    void prepareAction();
};


#endif // PARK_ACTION_CLIENT_H
