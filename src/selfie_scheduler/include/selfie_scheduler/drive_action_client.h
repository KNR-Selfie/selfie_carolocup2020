#ifndef DRIVE_ACTION_CLIENT_H
#define DRIVE_ACTION_CLIENT_H

#include <selfie_msgs/drivingAction.h>
#include <selfie_scheduler/client_interface.h>
#include <selfie_scheduler/scheduler_enums.h>

class DriveClient : public ClientInterface
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<selfie_msgs::drivingAction> ac_;
    selfie_msgs::drivingGoal goal_;
    program_state action_state_;
    bool result_flag_;

public:
    DriveClient(std::string name);
    ~DriveClient();

    void setGoal(boost::any goal);
    bool waitForResult(float timeout);
    void cancelAction();
    bool waitForServer(float timeout);

    program_state getActionState();
    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const selfie_msgs::drivingResultConstPtr& result);
    void activeCb();
    void feedbackCb(const selfie_msgs::drivingFeedbackConstPtr& feedback);
    bool getResult();
    bool isActionFinished();

};


#endif // DRIVE_ACTION_CLIENT_H
