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
    , proof_overtake_(0)
    , num_proof_to_overtake_(3)
    , num_corners_to_detect_(3)
    , current_distance_(0)
    , current_offset_(0)
    , return_distance_calculated_(false)
    , pos_tolerance_(0.01)
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
  pnh_.param<float>("pos_tolerance", pos_tolerance_, 0.01);
  pnh_.param<int>("num_proof_to_overtake", num_proof_to_overtake_, 3);
  pnh_.param<int>("num_corners_to_detect", num_corners_to_detect_, 3);
  
  passive_mode_service_ = nh_.advertiseService("/avoiding_obst_set_passive", &Road_obstacle_detector::switchToPassive, this);
  active_mode_service_ = nh_.advertiseService("/avoiding_obst_set_active", &Road_obstacle_detector::switchToActive, this);
  reset_node_service_ = nh_.advertiseService("/resetLaneControl", &Road_obstacle_detector::reset_node, this);
  setpoint_pub_ = nh_.advertise<std_msgs::Float64>("/setpoint", 1);
  speed_pub_ = nh_.advertise<std_msgs::Float64>("/max_speed", 1);
  right_indicator_pub_ = nh_.advertise<std_msgs::Bool>("right_turn_indicator", 20);
  left_indicator_pub_ = nh_.advertise<std_msgs::Bool>("left_turn_indicator", 20);
  
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
  if (status_ != OVERTAKE)
  {
    filter_boxes(msg);
    if (!filtered_boxes_.empty())
    {
      if (nearest_box_in_front_of_car_->bottom_left.x <= maximum_distance_to_obstacle_ ||
          nearest_box_in_front_of_car_->bottom_right.x <= maximum_distance_to_obstacle_)
      {
        ++proof_overtake_;
        if (proof_overtake_ >= num_proof_to_overtake_)
        {
          proof_overtake_ = 0;
          calculate_return_distance();
          ROS_INFO("LC: OVERTAKE");
          status_ = OVERTAKE;
        }
      } else
      {
        if (proof_overtake_ > 0)
        {
          --proof_overtake_;
        }
      }
    } else
    {
      if (proof_overtake_ > 0)
      {
        --proof_overtake_;
      }
    }
  }
  switch (status_)
  {
  case ON_RIGHT:
    setpoint_value_.data = right_lane_;

    if (proof_overtake_ == 0)
      speed_message_.data = max_speed_;
    else
      speed_message_.data = safe_speed_;

    break;
  case OVERTAKE:
    blinkRight(false);
    blinkLeft(true);
    setpoint_value_.data = left_lane_;
    speed_message_.data = safe_speed_;
    break;
  case ON_LEFT:
    setpoint_value_.data = left_lane_;

    if (proof_overtake_ == 0)
      speed_message_.data = max_speed_;
    else
      speed_message_.data = safe_speed_;

    break;
  case RETURN:
    blinkRight(true);
    blinkLeft(false);
    setpoint_value_.data = right_lane_;
    speed_message_.data = safe_speed_;
    break;
  case PASSIVE:
    return;
  default:
    ROS_ERROR("Wrong avoiding_obstacle action status");
  }
  setpoint_pub_.publish(setpoint_value_);
  speed_pub_.publish(speed_message_);
}

void Road_obstacle_detector::filter_boxes(const selfie_msgs::PolygonArray &msg)
{
  filtered_boxes_.clear();
  if (!msg.polygons.empty())
  {
    geometry_msgs::Polygon polygon;
    for (int box_nr = msg.polygons.size() - 1; box_nr >= 0; box_nr--)
    {
      polygon = msg.polygons[box_nr];
      int corners_ok = 0;
      for (int a = 0; a < 4; ++a)
      {
        Point p(polygon.points[a]);

        if (is_on_right_lane(p) && p.check_position(ROI_min_x_, ROI_max_x_, ROI_min_y_, ROI_max_y_))
        {
          ++corners_ok;
        }
      }
      if (corners_ok >= num_corners_to_detect_)
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
  else
    return false;
}

void Road_obstacle_detector::calculate_return_distance()
{
  return_distance_calculated_ = true;
  return_distance_ =
      safety_margin_ * (maximum_length_of_obstacle_ + nearest_box_in_front_of_car_->bottom_left.x) + current_distance_;
  ROS_INFO("LC: return_distance_: %f", return_distance_);
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
  if (return_distance_calculated_ && status_ != ON_RIGHT && current_distance_ > return_distance_)
  {
    status_ = RETURN;
    ROS_INFO("LC: RETURN");
    return_distance_calculated_ = false;
    selfie_msgs::PolygonArray temp;
    obstacle_callback(temp);
  }
}

void Road_obstacle_detector::posOffsetCallback(const std_msgs::Float64 &msg)
{
  current_offset_ = msg.data;
  if (status_ == OVERTAKE && current_offset_ > left_lane_ - pos_tolerance_)
  {
    status_ = ON_LEFT;
    ROS_INFO("LC: ON_LEFT");
    blinkLeft(false);
    selfie_msgs::PolygonArray temp;
    obstacle_callback(temp);
  }
  else if (status_ == RETURN && current_offset_ < right_lane_ + pos_tolerance_)
  {
    status_ = ON_RIGHT;
    ROS_INFO("LC: ON_RIGHT");
    blinkRight(false);
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
  pos_offset_sub_ = nh_.subscribe("/position_offset", 1, &Road_obstacle_detector::posOffsetCallback, this);
  blinkLeft(false);
  blinkRight(false);
  return_distance_calculated_ = false;
  proof_overtake_ = 0;
  timer_.stop();
  status_ = ON_RIGHT;
  ROS_INFO("Lane control active mode");
  return true;
}

bool Road_obstacle_detector::switchToPassive(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  markings_sub_.shutdown();
  obstacles_sub_.shutdown();
  distance_sub_.shutdown();
  pos_offset_sub_.shutdown();
  setpoint_value_.data = right_lane_;
  status_ = PASSIVE;
  timer_.start();
  ROS_INFO("Lane control passive mode");
  return true;
}

bool Road_obstacle_detector::reset_node(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  ROS_INFO("Lane control reset");
  if (status_ != PASSIVE)
    switchToActive(request, response);

  return true;
}

void Road_obstacle_detector::blinkLeft(bool on)
{
  std_msgs::Bool msg;
  msg.data = on;
  left_indicator_pub_.publish(msg);
  return;
}

void Road_obstacle_detector::blinkRight(bool on)
{
  std_msgs::Bool msg;
  msg.data = on;
  right_indicator_pub_.publish(msg);
  return;
}