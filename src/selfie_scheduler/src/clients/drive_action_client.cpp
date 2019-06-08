#include <selfie_scheduler/drive_action_client.h>

DriveClient::DriveClient(std::string name):
    ac_(name, true)
{
    ROS_INFO("Wait for driving action server");
     ac_.waitForServer();
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

void DriveClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const selfie_msgs::drivingResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("result: %d", result->parking_area);
}

void DriveClient::activeCb()
{
  ROS_INFO("Starting procedure server active");
  //STARTING
}
void DriveClient::feedbackCb(const selfie_msgs::drivingFeedbackConstPtr& feedback)
{
  ROS_INFO("Starting procedure feedback %d", feedback->action_status);
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
