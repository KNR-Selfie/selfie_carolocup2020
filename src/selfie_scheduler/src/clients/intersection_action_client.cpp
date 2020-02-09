#include <selfie_scheduler/intersection_action_client.h>
#include <std_srvs/Empty.h>

IntersectionClient::IntersectionClient(std::string name):
    ac_(name, true)
{
    result_flag_ = EMPTY;
    next_action_ = DRIVING;
    action_state_ = SELFIE_IDLE;
    avoidingObstSetPassive_ = nh_.serviceClient<std_srvs::Empty>("avoiding_obst_set_passive");
    resetLaneController_ = nh_.serviceClient<std_srvs::Empty>("resetLaneControl");
}
IntersectionClient::~IntersectionClient()
{
}
void IntersectionClient::setGoal(boost::any goal)
{
    ac_.sendGoal(goal_, boost::bind(&IntersectionClient::doneCb, this, _1, _2),
                boost::bind(&IntersectionClient::activeCb, this),
                boost::bind(&IntersectionClient::feedbackCb, this, _1));
}
bool IntersectionClient::waitForResult(float timeout)
{
    return ac_.waitForResult(ros::Duration(timeout));
}
bool IntersectionClient::waitForServer(float timeout)
{
    result_flag_ = EMPTY;
    ROS_INFO("Wait for itersection action server");
    return ac_.waitForServer(ros::Duration(timeout));
}
void IntersectionClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const selfie_msgs::intersectionResultConstPtr& result)
{
    ROS_INFO("Finished itersection in state [%s]", state.toString().c_str());
    ROS_INFO("itersection result: %d", result->done);
    if(state == State::ABORTED)
    {
        result_flag_ = ABORTED;
    }
    else if(state == State::SUCCEEDED)
    {
        result_ = result->done;
        result_flag_ = SUCCESS;
    }
}
void IntersectionClient::activeCb()
{
    ROS_INFO("itersection next_action_tion server active");
}
void IntersectionClient::feedbackCb(const selfie_msgs::intersectionFeedbackConstPtr& feedback)
{
  ROS_INFO("itersection action feedback %d", feedback->action_status);
  action_state_ = (program_state)feedback->action_status;
}
void IntersectionClient::cancelAction()
{
  ROS_INFO("itersection cancel action");
  ac_.cancelAllGoals();
}
void IntersectionClient::getActionResult(boost::any &result)
{
    // result = result_;
}
void IntersectionClient::prepareAction()
{
    std_srvs::Empty empty_msg;
    resetLaneController_.call(empty_msg);
    avoidingObstSetPassive_.call(empty_msg);
    ROS_INFO("Prepare intersection - call reset Lane control and avoiding obst passive");
}

