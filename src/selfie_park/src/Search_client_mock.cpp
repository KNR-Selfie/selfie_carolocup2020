/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include <selfie_park/Search_client_mock.h>

float Search_client_mock::last_speed_ = -1;

Search_client_mock::Search_client_mock(const ros::NodeHandle &nh)
    : nh_(nh)
    , ac_("search", true)
{
  ROS_INFO("Waiting for action server to start.");
  ac_.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  speed_subscriber_ = nh_.subscribe("/speed", 1, topicCallback);
}

void placeHolder(){};

void Search_client_mock::send_goal(const float spot_length = 0.5)
{
  last_speed_ = -1;
  selfie_msgs::searchGoal msg;
  msg.min_spot_lenght = spot_length;
  ac_.sendGoal(msg, boost::bind(&Search_client_mock::doneCb, this, _1, _2), &placeHolder, &feedbackCb);
}

void feedbackCb(const selfie_msgs::searchFeedbackConstPtr &feedback)
{
  std::cout << std::endl << "\n\nRecieved feedback: ";
  switch (feedback->action_status)
  {
  case START_SEARCHING_PLACE:
    std::cout << "START_SEARCHING_PLACE "
              << "\nLast recieved speed: " << Search_client_mock::last_speed_;
    break;
  case FOUND_PLACE_MEASURING:
    std::cout << "FOUND_PLACE_MEASURING "
              << "\nLast recieved speed: " << Search_client_mock::last_speed_;
    break;
  case FIND_PROPER_PLACE:
    std::cout << "FIND_PROPER_PLACE "
              << "\nLast recieved speed: " << Search_client_mock::last_speed_;
    break;

  default:
    std::cout << "ERR-Different status ";
    break;
  }
}

void topicCallback(const std_msgs::Float64ConstPtr &msg)
{
  Search_client_mock::last_speed_ = msg->data;
  std::cout << " \nNewSpeed: " << msg->data;
}

void Search_client_mock::doneCb(const actionlib::SimpleClientGoalState &state,
                                const selfie_msgs::searchResultConstPtr &result)
{
  ROS_INFO("Done, place is found \nfound place:");
  Box(result->parking_spot).print();
  ROS_INFO("Shuting down client node...");
  ros::shutdown();
}

void sendMockObstacles(const ros::TimerEvent &)
{
  selfie_msgs::PolygonArray obstacle_array_;
  geometry_msgs::Point32 p;
  p.z = 0;
  float mock_box_size_ = 0.3;
  float boxes_x[] = {0.1, 1.33, 1.6};
  float boxes_y[] = {-0.3, -0.35, -0.3};
  obstacle_array_.polygons.clear();
  obstacle_array_.header.stamp = ros::Time::now();
  obstacle_array_.header.frame_id = "laser";
  for (unsigned int i = 0; i < 3; i++)
  {
    geometry_msgs::Polygon obstacle;
    for (unsigned int ii = 0; ii < 4; ii++)
    {
      p.x = boxes_x[i];
      p.y = boxes_y[i];
      switch (ii)
      {
      case 0:
        p.x += mock_box_size_ / 2;
        p.y += mock_box_size_ / 2;
        break;
      case 1:
        p.x += mock_box_size_ / 2;
        p.y -= mock_box_size_ / 2;
        break;
      case 2:
        p.x -= mock_box_size_ / 2;
        p.y -= mock_box_size_ / 2;
        break;
      case 3:
        p.x -= mock_box_size_ / 2;
        p.y += mock_box_size_ / 2;
        break;
      }
      obstacle.points.push_back(p);
    }
    obstacle_array_.polygons.push_back(obstacle);
  }
  Search_client_mock::obstacles_pub_.publish(obstacle_array_);
  ROS_INFO_ONCE("Sending mock obstacles");
}
