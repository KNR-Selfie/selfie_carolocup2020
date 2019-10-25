#include <selfie_scheduler/drive_action_client.h>

DriveClient::DriveClient(std::string name):
    ac_(name, true)
{
    next_action_ = PARKING_SEARCH;
    action_state_ = SELFIE_IDLE;
    result_flag_ = EMPTY;
}
DriveClient::~DriveClient()
{
}
void DriveClient::setGoal(boost::any goal)
{
    float drive_mode;

    try
    {
        drive_mode = boost::any_cast<bool>(goal);
    }
    catch (boost::bad_any_cast &e)
    {
        ROS_ERROR("bad casting %s", e.what());
        return;
    }

    // select next action based on button pressed
    if(drive_mode == false)
        next_action_ = INTERSECTION;
    else
        next_action_ = PARKING_SEARCH;

    goal_.mode = drive_mode;
    ac_.sendGoal(goal_, boost::bind(&DriveClient::doneCb, this, _1, _2),
                boost::bind(&DriveClient::activeCb, this),
                boost::bind(&DriveClient::feedbackCb, this, _1));
}
bool DriveClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}
bool DriveClient::waitForServer(float timeout)
{
    result_flag_ = EMPTY;
    ROS_INFO("Wait for driving action server");
    return ac_.waitForServer(ros::Duration(timeout));
}
void DriveClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const selfie_msgs::drivingResultConstPtr& result)
{
    ROS_INFO("Finished drive in state [%s]", state.toString().c_str());
    ROS_INFO("drive result: %d", result->parking_area);

    if(state == State::ABORTED)
    {
        result_flag_ = ABORTED;
    }
    else
    {
        result_ = result->event;
	result_flag_ = SUCCESS;
    }
}
void DriveClient::activeCb()
{
    ROS_INFO("Drive acnext_action_tion server active");
}
void DriveClient::feedbackCb(const selfie_msgs::drivingFeedbackConstPtr& feedback)
{
  ROS_INFO("Drive action feedback %d", feedback->action_status);
  action_state_ = (program_state)feedback->action_status;
}
void DriveClient::cancelAction()
{
  ROS_INFO("Drive cancel action");
  ac_.cancelAllGoals();
}
program_state DriveClient::getActionState()
{
    if(action_state_ != SELFIE_IDLE)
        return action_state_;
}
int DriveClient::isActionFinished()
{
    return result_flag_;
}
void DriveClient::getActionResult(boost::any &result)
{
    // result = result_;
}
action DriveClient::getNextAction()
{
    return next_action_;
}
