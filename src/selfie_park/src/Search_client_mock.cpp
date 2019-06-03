#include "../../selfie_park/include/selfie_park/Search_client_mock.h"

float Search_client_mock::last_speed_ = -1;

Search_client_mock::Search_client_mock(const ros::NodeHandle &nh)
    : nh_(nh), ac_("search", true) {
  ROS_INFO("Waiting for action server to start.");
  ac_.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  speed_subscriber_ = nh_.subscribe("/speed", 1, topicCallback);
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
    std::cout << "START_SEARCHING_PLACE "<< "\nLast recieved speed: " << Search_client_mock::last_speed_;
    break;
  case FIND_PLACE:
    std::cout << "FIND_PLACE "<< "\nLast recieved speed: " << Search_client_mock::last_speed_;
    break;
  case FIND_PROPER_PLACE:
    std::cout << "FIND_PROPER_PLACE "<< "\nLast recieved speed: " << Search_client_mock::last_speed_;
    break;

  default:
    std::cout << "ERR-Different status ";
    break;
  }
}

void topicCallback(const std_msgs::Float64ConstPtr &msg) {
   Search_client_mock::last_speed_ = msg->data;
   std::cout<<" \nNewSpeed: "<<msg->data;
}

void Search_client_mock::doneCb(const actionlib::SimpleClientGoalState& state,const selfie_msgs::searchResultConstPtr& result)
{
  ROS_INFO("Done, place is found \nfound place:");
  Box(result->parking_spot).print_box_dimensions();
  ROS_INFO("Shuting down client node...");
  ros::shutdown();
  
}