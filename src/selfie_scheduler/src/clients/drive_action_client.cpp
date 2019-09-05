#include <selfie_scheduler/drive_action_client.h>

DriveClient::DriveClient(std::string name):
    ac_(name, true)
{
    result_flag_ = false;
    next_action_ = PARKING_SEARCH;
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
        ROS_ERROR("bad casting %s",e.what());
        return;
    }

    ROS_INFO("Good goal cast");
    goal_.mode = drive_mode;
    ac_.sendGoal(goal_,boost::bind(&DriveClient::doneCb, this, _1,_2),
                boost::bind(&DriveClient::activeCb,this),
                boost::bind(&DriveClient::feedbackCb,this,_1));
}

bool DriveClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}
bool DriveClient::waitForServer(float timeout)
{
    ROS_INFO("Wait for driving action server");
    return ac_.waitForServer(ros::Duration(timeout));
}
void DriveClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const selfie_msgs::drivingResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("result: %d", result->parking_area);
    result_ = result->parking_area;
    result_flag_ = true;
}

void DriveClient::activeCb()
{
    ROS_INFO("Drive action server active");
}
void DriveClient::feedbackCb(const selfie_msgs::drivingFeedbackConstPtr& feedback)
{
  ROS_INFO("Drive action feedback %d", feedback->action_status);
  action_state_ = (program_state)feedback->action_status;
}

void DriveClient::cancelAction()
{
  ac_.cancelAllGoals();
}

program_state DriveClient::getActionState()
{
    return action_state_;
}

bool DriveClient::isActionFinished()
{
    return result_flag_;
}
void DriveClient::getActionResult(boost::any &result)
{
   //empty implementation
}
action DriveClient::getNextAction()
{
    return next_action_;
}
