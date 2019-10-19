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
  obstacles_sub_ = nh_.subscribe("/obstacles", 1, &Road_obstacle_detector::obstacle_callback, this);
  markings_sub_ = nh_.subscribe("/road_markings", 1, &Road_obstacle_detector::road_markings_callback, this);
  setpoint_pub_ = nh_.advertise<std_msgs::Float32>("/setpoint", 1);

  if (visualization_)
  {
    visualizer_ = nh_.advertise<visualization_msgs::Marker>("/avoiding_obstacles", 1);
  }
  status_ = CLEAR;
  ROS_INFO("road_obstacle_detector initialized ");
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
        change_lane(LEFT);
        status_ = OVERTAKING;
        speed_sub_ = nh_.subscribe("/speed", 1, &Road_obstacle_detector::calculate_overtake_time, this);
      }
    break;
  case OVERTAKING:
    if (time_left_ <= 0 && is_time_calculated_for_overtake_)
    {
      change_lane(RIGHT);
      status_ = CLEAR;
      is_time_calculated_for_overtake_ = false;
    }
    break;
  default:
    ROS_ERROR("Wrong avoiding_obstacle action status");
  }
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

      if (is_on_right_lane(p))
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

void Road_obstacle_detector::calculate_overtake_time(const std_msgs::Float32 &msg)
{
  time_left_ = (maximum_length_of_obstacle_ + maximum_distance_to_obstacle_) / msg.data;
  timer_ = nh_.createTimer(ros::Duration(timer_duration_), &Road_obstacle_detector::calculate_time, this);
  is_time_calculated_for_overtake_ = true;
  speed_sub_.shutdown();
}

void Road_obstacle_detector::change_lane(float lane)
{
  setpoint_value_.data = lane;
  setpoint_pub_.publish(setpoint_value_);
  if (lane == LEFT)
    ROS_INFO("Lane changed to left");
  else
    ROS_INFO("Lane changed to right");
}

void Road_obstacle_detector::calculate_time(const ros::TimerEvent &time) { time_left_ -= timer_duration_; }
