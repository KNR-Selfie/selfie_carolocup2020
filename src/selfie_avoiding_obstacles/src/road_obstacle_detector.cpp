/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include <selfie_avoiding_obstacles/road_obstacle_detector.h>

Road_obstacle_detector::Road_obstacle_detector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh)
    , pnh_(pnh)
    , timer_duration_(0.3)
    , is_time_calculated_for_overtake_(false)
    , received_road_markings_(false)
    , maximum_distance_to_obstacle_(0.5)
{
  pnh_.param<bool>("visualization", visualization_, false);
  pnh_.param<float>("maximum_length_of_obstacle", maximum_length_of_obstacle_, 0.8);
  obstacles_sub_ = nh_.subscribe("/obstacles", 1, &Road_obstacle_detector::obstacle_callback, this);
  passive_mode_service_ = nh_.advertiseService("/avoiding_obst_set_passive", &Road_obstacle_detector::switchToPassive, this);
  active_mode_service_ = nh_.advertiseService("/avoiding_obst_set_active", &Road_obstacle_detector::switchToActive, this);
  setpoint_pub_ = nh_.advertise<std_msgs::Float32>("/setpoint", 1);
  speed_pub_ = nh_.advertise<std_msgs::Float32>("/speed", 1);
  pnh_.param<float>("point_min_x", point_min_x_, 0.3);
  pnh_.param<float>("point_max_x", point_max_x_, 1.1);
  pnh_.param<float>("point_min_y", point_min_y_, -1.3);
  pnh_.param<float>("point_max_y", point_max_y_, 1.3);
  pnh_.param<float>("right_lane_setpoint", right_lane_, -0.2);
  pnh_.param<float>("left_lane_setpoint", left_lane_, 0.2);
  pnh_.param<float>("deafult_setpoint", default_setpoint_, -0.2);
  pnh_.param<float>("maximum_speed", max_speed_, 0.3);
  pnh_.param<float>("safety_margin", safety_margin_, 1.15);
  speed_message_.data = max_speed_;

  if (visualization_)
  {
    visualizer_ = nh_.advertise<visualization_msgs::Marker>("/avoiding_obstacles", 1);
  }
  status_ = PASSIVE;
  ROS_INFO("road_obstacle_detector initialized ");
  calculate_overtake_time();
}

Road_obstacle_detector::~Road_obstacle_detector() {}

void Road_obstacle_detector::obstacle_callback(const selfie_msgs::PolygonArray &msg)
{
  switch (status_)
  {
  case CLEAR:
    filter_boxes(msg);
    if (!filtered_boxes_.empty())
      if (nearest_box_in_front_of_car_->bottom_left.x <= maximum_distance_to_obstacle_)
      {
        change_lane(left_lane_);
        status_ = OVERTAKING;
      }
    break;
  case OVERTAKING:
    if (time_left_ <= 0)
    {
      change_lane(right_lane_);
      status_ = CLEAR;
      timer_.stop();
    }
    break;
  case PASSIVE:
    setpoint_value_.data = default_setpoint_;
    setpoint_pub_.publish(setpoint_value_);
    break;
  default:
    ROS_ERROR("Wrong avoiding_obstacle action status");
  }

  speed_pub_.publish(speed_message_);
}

void Road_obstacle_detector::filter_boxes(const selfie_msgs::PolygonArray &msg)
{
  filtered_boxes_.clear();
  geometry_msgs::Polygon polygon;
  for (int box_nr = msg.polygons.size() - 1; box_nr >= 0; box_nr--)
  {
    polygon = msg.polygons[box_nr];
    bool box_ok = false;
    for (int a = 0; a < 4; ++a)
    {
      Point p(polygon.points[a]);

      if (is_on_right_lane(p) && p.check_position(point_min_x_, point_max_x_, point_min_y_, point_max_y_))
      {
        box_ok = true;
        break;
      }
    }
    if (box_ok)
    {
      Box temp_box(polygon);
      filtered_boxes_.insert(filtered_boxes_.begin(), temp_box);
      if (temp_box.bottom_left.x > 0)
      {
        nearest_box_in_front_of_car_ = filtered_boxes_.begin();
      }
    }
  }
  if (visualization_)
  {
    visualizeBoxes();
  }
}

void Road_obstacle_detector::road_markings_callback(const selfie_msgs::RoadMarkings &msg)
{
  int size = msg.left_line.size();
  if (size != 3 && size != 4)
    ROS_ERROR("Invalid number of args in RoadMarkings");
  for (int i = 0; i < size; i++)
  {
    left_line_[i] = msg.left_line[i];
    center_line_[i] = msg.center_line[i];
    right_line_[i] = msg.right_line[i];
  }
  if (size == 3)
  {
    left_line_[3] = 0;
    center_line_[3] = 0;
    right_line_[3] = 0;
  }
  received_road_markings_ = true;
}

bool Road_obstacle_detector::is_on_right_lane(const Point &point)
{
  if (received_road_markings_ == false)
    return false;

  float right_value = right_line_[0] + point.x * right_line_[1] + point.x * point.x * right_line_[2] +
                      point.x * point.x * point.x * right_line_[3];
  float center_value = center_line_[0] + point.x * center_line_[1] + point.x * point.x * center_line_[2] +
                       point.x * point.x * point.x * center_line_[3];

  if (point.y > right_value && point.y < center_value)
    return true;
}

void Road_obstacle_detector::calculate_overtake_time()
{
  time_left_ = safety_margin_ * (maximum_length_of_obstacle_ + maximum_distance_to_obstacle_) / max_speed_;
  timer_ = nh_.createTimer(ros::Duration(timer_duration_), &Road_obstacle_detector::calculate_time, this);
}

void Road_obstacle_detector::change_lane(float lane)
{
  setpoint_value_.data = lane;
  setpoint_pub_.publish(setpoint_value_);
  if (lane == left_lane_)
    ROS_INFO("Lane changed to left");
  else
    ROS_INFO("Lane changed to right");
}

void Road_obstacle_detector::visualizeBoxes()
{
  Box().visualizeList(filtered_boxes_, visualizer_, "boxes_on_lane", 0.9, 0.9, 0.9);
  nearest_box_in_front_of_car_->visualize(visualizer_, "nearest_box", 1, 0.1, 0.1);
}

void Road_obstacle_detector::calculate_time(const ros::TimerEvent &time) { time_left_ -= timer_duration_; }

bool Road_obstacle_detector::switchToActive(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  markings_sub_ = nh_.subscribe("/road_markings", 1, &Road_obstacle_detector::road_markings_callback, this);
  status_ = CLEAR;
  return true;
}

bool Road_obstacle_detector::switchToPassive(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  markings_sub_.shutdown();
  status_ = PASSIVE;
  return true;
}