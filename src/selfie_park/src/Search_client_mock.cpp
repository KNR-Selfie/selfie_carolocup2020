#include "../../selfie_park/include/selfie_park/Search_client_mock.h"

float Search_client_mock::last_speed_ = -1;

Search_client_mock::Search_client_mock(const ros::NodeHandle &nh)
    : nh_(nh), ac_("search", true) {
  ROS_INFO("Waiting for action server to start.");
  ac_.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  speed_subscriber_ = nh_.subscribe("/speed", 2, topicCallback);
}

void placeHolder(){};

void Search_client_mock::send_goal(const float spot_length = 0.5) {
  last_speed_ = -1;
  selfie_msgs::searchGoal msg;
  msg.min_spot_lenght = spot_length; 
  ac_.sendGoal(msg,boost::bind(&Search_client_mock::doneCb, this, _1, _2),&placeHolder,&feedbackCb);
}

void feedbackCb(const selfie_msgs::searchFeedbackConstPtr &feedback) {
  std::cout << std::endl << "\n\nRecieved feedback: ";
  switch (feedback->action_status) {
  case START_SEARCHING_PLACE:
    std::cout << "START_SEARCHING_PLACE ";
    break;
  case FIND_PLACE:
    std::cout << "FIND_PLACE ";
    break;
  case FIND_PROPER_PLACE:
    std::cout << "FIND_PROPER_PLACE ";
    break;

  default:
    std::cout << "ERR-Different status ";
  }
   std::cout << "\nLast recieved speed" << Search_client_mock::last_speed_;
}

void topicCallback(const std_msgs::Float64ConstPtr &msg) {
   Search_client_mock::last_speed_ = msg->data;
}