/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include <selfie_intersection/intersection_server.h>

IntersectionServer::IntersectionServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh)
    , pnh_(pnh)
    , intersectionServer_(nh_, "intersection", false)
    , point_max_x_(0.95) // Width of road
    , dr_server_CB_(boost::bind(&IntersectionServer::reconfigureCB, this, _1, _2))
    , is_distance_to_intersection_saved_(false)
    , is_distance_saved_(false)
{
  intersectionServer_.registerGoalCallback(boost::bind(&IntersectionServer::init, this));
  intersectionServer_.registerPreemptCallback(boost::bind(&IntersectionServer::preemptCb, this));
  intersectionServer_.start();
  dr_server_.setCallback(dr_server_CB_);
  pnh_.param<float>("distance_to_intersection", max_distance_to_intersection_, 0.7);
  pnh_.param<float>("road_width", road_width_, 0.95);
  pnh_.param<float>("point_min_y", point_min_y_, -2);
  pnh_.param<float>("point_max_y", point_max_y_, 2);
  pnh_.param<float>("stop_time", stop_time_, 3);
  pnh_.param<float>("speed_default", speed_default_, 0.3);
  pnh_.param<float>("distance_of_blind_approaching", distance_of_blind_approaching_, 0.3);
  pnh_.param<int>("num_corners_to_detect", num_corners_to_detect_, 3);
  pnh_.param<bool>("visualization", visualization_, true);
  point_min_x_ = max_distance_to_intersection_;
  point_max_x_ = point_min_x_ + road_width_;
  speed_.data = speed_default_;
  ROS_INFO("Intersection server initialized");

  if (visualization_)
  {
    visualize_intersection_ = nh_.advertise<visualization_msgs::Marker>("/intersection_visualization", 10);
  }
}

void IntersectionServer::init()
{
  goal_ = *(intersectionServer_.acceptNewGoal());
  obstacles_sub_ = nh_.subscribe("/obstacles", 1, &IntersectionServer::manager, this);
  speed_publisher_ = nh_.advertise<std_msgs::Float64>("/max_speed", 2);
  intersection_subscriber_ = nh_.subscribe("/intersection_distance", 1, &IntersectionServer::intersection_callback, this);
  distance_subscriber_ = nh_.subscribe("/distance", 1, &IntersectionServer::distance_callback, this);
  speed_.data = speed_default_;
  speed_publisher_.publish(speed_);
  publishFeedback(APPROACHING_TO_INTERSECTION);
  time_started_ = false;
  approached_blindly_ = false;
  ROS_INFO("Goal received - node activated");

  if (visualization_)
  {
    Box(point_min_x_, point_max_x_, point_min_y_, point_max_y_)
        .visualize(visualize_intersection_, "area_of_interest", 0.9, 0.9, 0.1);
  }
}

void IntersectionServer::manager(const selfie_msgs::PolygonArray &boxes)
{
  if (!intersectionServer_.isActive())
  {
    ROS_INFO_THROTTLE(2, "Intersection Server server not active");
    return;
  }
  filter_boxes(boxes);
  if (max_distance_to_intersection_ < point_min_x_)
  {
    speed_.data = speed_default_;
    speed_publisher_.publish(speed_);
    publishFeedback(APPROACHING_TO_INTERSECTION);
  } else
  {
    if (!time_started_)
    {
      beginning_time_ = ros::Time::now().toSec();
      time_started_ = true;
    }

    if (filtered_boxes_.size() != 0)
    {
      ROS_INFO_THROTTLE(1.5, "Another car on the road");
      if (visualization_)
      {
        Box().visualizeList(filtered_boxes_, visualize_intersection_, "obstacles_on_road", 0.9, 0.9, 0.9);
        Box(point_min_x_, point_max_x_, point_min_y_, point_max_y_)
            .visualize(visualize_intersection_, "area_of_interest", 0.9, 0.9, 0.1, 3);
      }
      if (action_status_.action_status != STOPPED_ON_INTERSECTION)
      {
        speed_.data = 0;
        speed_publisher_.publish(speed_);
        publishFeedback(STOPPED_ON_INTERSECTION);
      }
    } else
    {
      current_time_ = ros::Time::now().toSec();
      difftime_ = current_time_ - beginning_time_;
      if (difftime_ >= stop_time_)
      {
        publishFeedback(ROAD_CLEAR);
        ROS_INFO("Road clear, intersection action finished");
        send_goal();
      } else
      {
        if (approached_blindly_)
        {
          speed_.data = 0;
          speed_publisher_.publish(speed_);
          publishFeedback(WAITING_ON_INTERSECTION);
          ROS_INFO_THROTTLE(0.3, "Waiting (%lf s left) on intersection", stop_time_ - difftime_);
        } else
        {
          if (action_status_.action_status != APPROACHING_TO_INTERSECTION2)
          {
            distance_to_stop_blind_approaching_ = current_distance_ + distance_of_blind_approaching_;
            ROS_INFO("Approaching blindly");
          }

          speed_.data = speed_default_;
          speed_publisher_.publish(speed_);
          publishFeedback(APPROACHING_TO_INTERSECTION2);
        }
      }
    }
  }
}

void IntersectionServer::intersection_callback(const std_msgs::Float32 &msg)
{
  if (action_status_.action_status != APPROACHING_TO_INTERSECTION2)
  {
    point_min_x_ = msg.data;
    if (!is_distance_to_intersection_saved_)
    {
      distance_to_intersection_when_started_ = msg.data;
      is_distance_to_intersection_saved_ = true;
    }
    point_max_x_ = point_min_x_ + road_width_;
    if (intersectionServer_.isActive())
      ROS_INFO_THROTTLE(1, "Distance to intersection: %lf", point_min_x_);
  
    if (max_distance_to_intersection_ >= point_min_x_)
    {
      selfie_msgs::PolygonArray emptyBoxes;
      manager(emptyBoxes);
    }
  }
}

void IntersectionServer::distance_callback(const std_msgs::Float32 &msg)
{
  current_distance_ = msg.data;
  if (!is_distance_saved_)
  {
    is_distance_saved_ = true;
    distance_when_started_ = msg.data;
  }
  if (is_distance_to_intersection_saved_)
  {
    if (msg.data > distance_when_started_ + distance_to_intersection_when_started_)
    {
      ROS_INFO("Timeout for intersection (distance exceeded)");
      send_goal();
    }
  }
  if (!approached_blindly_ && action_status_.action_status == APPROACHING_TO_INTERSECTION2)
  {
    if (current_distance_ >= distance_to_stop_blind_approaching_)
    {
      approached_blindly_ = true;
    }
  }
}

void IntersectionServer::send_goal()
{
  selfie_msgs::intersectionResult result;
  result.done = true;
  is_distance_to_intersection_saved_ = false;
  is_distance_saved_ = false;
  point_min_x_ = max_distance_to_intersection_;
  point_max_x_ = point_min_x_ + road_width_;

  intersection_subscriber_.shutdown();
  distance_subscriber_.shutdown();
  obstacles_sub_.shutdown();
  intersectionServer_.setSucceeded();
}

void IntersectionServer::filter_boxes(const selfie_msgs::PolygonArray &msg)
{
  filtered_boxes_.clear();
  if (!msg.polygons.empty())
  {
    for (int box_nr = msg.polygons.size() - 1; box_nr >= 0; box_nr--)
    {
      geometry_msgs::Polygon polygon = msg.polygons[box_nr];
      int corners = 0;
      for (int a = 0; a < 4; ++a)
      {
        Point p(polygon.points[a]);
        if (p.check_position(point_min_x_, point_max_x_, point_min_y_, point_max_y_))
        {
          corners++;
        }
      }
      if (corners >= num_corners_to_detect_)
      {
        Box temp_box(polygon);
        filtered_boxes_.push_back(temp_box);
        if (visualization_ == false)
        {
          return; // for better optimalization
        }
      }
    }
  }
}

void IntersectionServer::publishFeedback(program_state newStatus)
{
  action_status_.action_status = newStatus;
  intersectionServer_.publishFeedback(action_status_);
}

void IntersectionServer::preemptCb()
{
  is_distance_saved_ = false;
  is_distance_to_intersection_saved_ = false;
  ROS_INFO("Intersection action preempted");
  distance_subscriber_.shutdown();
  obstacles_sub_.shutdown();
  intersection_subscriber_.shutdown();
  intersectionServer_.setAborted();
}

void IntersectionServer::reconfigureCB(selfie_intersection::IntersectionServerConfig &config, uint32_t level)
{
  if (max_distance_to_intersection_ != (float)config.distance_to_intersection)
  {
    max_distance_to_intersection_ = (float)config.distance_to_intersection;
    ROS_INFO("New max_distance_to_intersection_ value %f", max_distance_to_intersection_);
  }
  if (num_corners_to_detect_ != config.num_corners_to_detect)
  {
    num_corners_to_detect_ = config.num_corners_to_detect;
    ROS_INFO("New num_corners_to_detect_ value %d", num_corners_to_detect_);
  }
  if (point_max_y_ != (float)config.point_max_y)
  {
    point_max_y_ = (float)config.point_max_y;
    ROS_INFO("New point_max_y_ value %f", point_max_y_);
  }
  if (point_min_y_ != (float)config.point_min_y)
  {
    point_min_y_ = (float)config.point_min_y;
    ROS_INFO("New point_min_y value %f", point_min_y_);
  }
  if (speed_default_ != (float)config.speed_default)
  {
    speed_default_ = (float)config.speed_default;
    ROS_INFO("New speed_default value %f", speed_default_);
  }
  if (stop_time_ != (float)config.stop_time)
  {
    stop_time_ = (float)config.stop_time;
    ROS_INFO("New stop_time value %f", stop_time_);
  }
  if (distance_of_blind_approaching_ != (float)config.distance_of_blind_approaching)
  {
    distance_of_blind_approaching_ = (float)config.distance_of_blind_approaching;
    ROS_INFO("New distance_of_blind_approaching value %f", distance_of_blind_approaching_);
  }
}