/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include <selfie_avoiding_obstacles/road_obstacle_detector.h>

Road_obstacle_detector::Road_obstacle_detector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh)
    , pnh_(pnh)
    , received_road_markings_(false)
    , maximum_distance_to_obstacle_(0.5)
{
  pnh_.param<bool>("visualization", visualization_, false);
  pnh_.param<float>("maximum_length_of_obstacle", maximum_length_of_obstacle_, 0.8);
  pnh_.param<float>("maximum_distance_to_obstacle", maximum_distance_to_obstacle_, 0.5);
  pnh_.param<float>("ROI_min_x", ROI_min_x_, 0.3);
  pnh_.param<float>("ROI_max_x", ROI_max_x_, 1.1);
  pnh_.param<float>("ROI_min_y", ROI_min_y_, -1.3);
  pnh_.param<float>("ROI_max_y", ROI_max_y_, 1.3);
  pnh_.param<float>("right_lane_setpoint", right_lane_, -0.2);
  pnh_.param<float>("left_lane_setpoint", left_lane_, 0.2);
  pnh_.param<float>("maximum_speed", max_speed_, 0.3);
  pnh_.param<float>("safe_speed", safe_speed_, 0.1);
  pnh_.param<float>("safety_margin", safety_margin_, 1.15);
  
  passive_mode_service_ = nh_.advertiseService("/avoiding_obst_set_passive", &Road_obstacle_detector::switchToPassive, this);
  active_mode_service_ = nh_.advertiseService("/avoiding_obst_set_active", &Road_obstacle_detector::switchToActive, this);
  reset_node_service_ = nh_.advertiseService("/resetLaneControl", &Road_obstacle_detector::reset_node, this);
  setpoint_pub_ = nh_.advertise<std_msgs::Float64>("/setpoint", 1);
  speed_pub_ = nh_.advertise<std_msgs::Float64>("/max_speed", 1);
  
  speed_message_.data = max_speed_;

  if (visualization_)
  {
    visualizer_ = nh_.advertise<visualization_msgs::Marker>("/avoiding_obstacles", 1);
    // Initializing empty_marker_
    empty_marker_.header.frame_id = "laser";
    empty_marker_.header.stamp = ros::Time::now();
    empty_marker_.ns = "nearest_box";
    empty_marker_.type = visualization_msgs::Marker::LINE_LIST;
    empty_marker_.action = visualization_msgs::Marker::ADD;
    empty_marker_.id = 0;
    empty_marker_.lifetime = ros::Duration();
    // Initializing Box describing area of interest
    area_of_interest_box_ = Box(Point(ROI_min_x_, ROI_max_y_), Point(ROI_min_x_, ROI_min_y_), Point(ROI_max_x_, ROI_max_y_),
                                Point(ROI_max_x_, ROI_min_y_));
  }
  status_ = PASSIVE;
  setpoint_value_.data = right_lane_;
  timer_ = nh_.createTimer(ros::Duration(0.5), &Road_obstacle_detector::passive_timer_cb, this);
  timer_.start();

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
    {
      speed_message_.data = safe_speed_;
      if (nearest_box_in_front_of_car_->bottom_left.x <= maximum_distance_to_obstacle_)
      {
        change_lane(left_lane_);
        calculate_return_distance();
        status_ = OVERTAKING;
      } else
      {
        setpoint_value_.data = right_lane_;
      }
    } else
    {
      setpoint_value_.data = right_lane_;
      speed_message_.data = max_speed_;
    }
    setpoint_pub_.publish(setpoint_value_);
    break;
  case OVERTAKING:
    if (return_distance_ - current_distance_ <= 0)
    {
      speed_message_.data = safe_speed_;
      change_lane(right_lane_);
      status_ = CLEAR;
    } else
    {
      setpoint_value_.data = left_lane_;
      speed_message_.data = max_speed_;
    }
    setpoint_pub_.publish(setpoint_value_);
    break;
  case PASSIVE:
    setpoint_value_.data = right_lane_;
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

      if (is_on_right_lane(p) && p.check_position(ROI_min_x_, ROI_max_x_, ROI_min_y_, ROI_max_y_))
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

void Road_obstacle_detector::calculate_return_distance()
{
  return_distance_ =
      safety_margin_ * (maximum_length_of_obstacle_ + nearest_box_in_front_of_car_->bottom_left.x) + current_distance_;
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
  area_of_interest_box_.visualize(visualizer_, "area_of_interest", 1, 1, 1);
  if (filtered_boxes_.empty())
  {
    visualizer_.publish(empty_marker_);
  } else
    nearest_box_in_front_of_car_->visualize(visualizer_, "nearest_box", 1, 0.1, 0.1);
}

void Road_obstacle_detector::distanceCallback(const std_msgs::Float32 &msg)
{
  current_distance_ = msg.data;
  if (status_ == OVERTAKING && current_distance_ > return_distance_)
  {
    selfie_msgs::PolygonArray temp;
    obstacle_callback(temp);
  }
}

void Road_obstacle_detector::passive_timer_cb(const ros::TimerEvent &time) { setpoint_pub_.publish(setpoint_value_); }

bool Road_obstacle_detector::switchToActive(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  obstacles_sub_ = nh_.subscribe("/obstacles", 1, &Road_obstacle_detector::obstacle_callback, this);
  markings_sub_ = nh_.subscribe("/road_markings", 1, &Road_obstacle_detector::road_markings_callback, this);
  distance_sub_ = nh_.subscribe("/distance", 1, &Road_obstacle_detector::distanceCallback, this);
  timer_.stop();
  status_ = CLEAR;
  return true;
}

bool Road_obstacle_detector::switchToPassive(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  markings_sub_.shutdown();
  obstacles_sub_.shutdown();
  distance_sub_.shutdown();
  setpoint_value_.data = right_lane_;
  status_ = PASSIVE;
  timer_.start();
  return true;
}

bool Road_obstacle_detector::reset_node(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  if (status_ != PASSIVE)
    switchToActive(request, response);

  return true;
}
