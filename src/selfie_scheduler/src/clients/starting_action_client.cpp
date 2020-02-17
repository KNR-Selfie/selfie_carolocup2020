#include <selfie_scheduler/starting_action_client.h>

StartingProcedureClient::StartingProcedureClient(std::string name):
    ac_(name, true)
{
    result_flag_ = EMPTY;
    next_action_ = DRIVING;
    action_state_ = SELFIE_IDLE;
}
StartingProcedureClient::~StartingProcedureClient()
{
}
void StartingProcedureClient::setGoal(boost::any goal)
{
    float distance;

    try
    {
        distance = boost::any_cast<float>(goal);
    }
    catch (boost::bad_any_cast &e)
    {
        ROS_ERROR("bad casting %s", e.what());
        return;
    }
    goal_.distance = distance;
    ac_.sendGoal(goal_, boost::bind(&StartingProcedureClient::doneCb, this, _1, _2),
                boost::bind(&StartingProcedureClient::activeCb, this),
                boost::bind(&StartingProcedureClient::feedbackCb, this, _1));
}

bool StartingProcedureClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}
bool StartingProcedureClient::waitForServer(float timeout)
{
    result_flag_ = EMPTY;
    ROS_INFO("Wait for starting procedure action server");
    return ac_.waitForServer(ros::Duration(timeout));
}
void StartingProcedureClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const selfie_msgs::startingResultConstPtr& result)
{
  ROS_INFO("Finished starting in state [%s]", state.toString().c_str());
  if(state == actionlib::SimpleClientGoalState::StateEnum::ABORTED)
  {
      result_flag_ = ABORTED;
  }
  else
  {
      ROS_INFO("starting result: %i", result->drive_mode);
      result_ = result->drive_mode;
      result_flag_ = SUCCESS;
  }
}

void StartingProcedureClient::activeCb()
{
  ROS_INFO("Starting procedure server active");
}
void StartingProcedureClient::feedbackCb(const selfie_msgs::startingFeedbackConstPtr& feedback)
{
  ROS_INFO("Starting procedure feedback %d", feedback->action_status);
  action_state_ = (program_state)feedback->action_status;
}
void StartingProcedureClient::cancelAction()
{
  ac_.cancelAllGoals();
}
void StartingProcedureClient::getActionResult(boost::any &result)
{
    result = result_;
}
void StartingProcedureClient::prepareAction()
{
    
}