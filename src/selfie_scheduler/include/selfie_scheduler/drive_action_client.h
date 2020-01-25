#ifndef DRIVE_ACTION_CLIENT_H
#define DRIVE_ACTION_CLIENT_H

#include <selfie_msgs/drivingAction.h>
#include <selfie_scheduler/client_interface.h>
#include <selfie_scheduler/scheduler_enums.h>

class DriveClient : public ClientInterface
{
protected:
    actionlib::SimpleActionClient<selfie_msgs::drivingAction> ac_;
    selfie_msgs::drivingGoal goal_;
    bool result_;
    bool drive_mode_;
    int drive_steering_mode_;
    bool park_complete_;

    ros::ServiceClient visionReset_ ;
    ros::ServiceClient resetLaneController_;
    ros::ServiceClient cmdCreatorStartPub_;
    ros::ServiceClient steeringModeSetAckermann_;
    ros::ServiceClient steeringModeSetParallel_;
    ros::ServiceClient steeringModeSetFrontAxis_;
    ros::ServiceClient avoidingObstSetPassive_;
    ros::ServiceClient avoidingObstSetActive_;

    ros::NodeHandle pnh_;
    void checkParkCounter(boost::any goal);

public:
    DriveClient(std::string name, const ros::NodeHandle &pnh);
    ~DriveClient();

    void setDriveMode(bool drive_mode);
    void setGoal(boost::any goal);
    bool waitForResult(float timeout);
    void cancelAction();
    bool waitForServer(float timeout);

    void doneCb(const actionlib::SimpleClientGoalState& state,
                    const selfie_msgs::drivingResultConstPtr& result);
    void activeCb();
    void feedbackCb(const selfie_msgs::drivingFeedbackConstPtr& feedback);
    void getActionResult(boost::any &result);
    void setDriveSteeringMode();
    void prepareAction();
};


#endif // DRIVE_ACTION_CLIENT_H
