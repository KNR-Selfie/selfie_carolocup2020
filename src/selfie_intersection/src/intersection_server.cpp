/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include <selfie_intersection/intersection_server.h>

IntersectionServer::IntersectionServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh)
    , pnh_(pnh)
    , intersectionServer_(nh_, "intersection", false)
    , point_max_x_(0.95)
{
  intersectionServer_.registerGoalCallback(boost::bind(&IntersectionServer::init, this));
  intersectionServer_.start();
  speed_.data = 0;
  pnh_.param<float>("distance_to_intersection", max_distance_to_intersection_, 0.25);
  pnh_.param<float>("road_width", road_width_, 0.95);
  pnh_.param<float>("point_min_y", point_min_y_, -3);
  pnh_.param<float>("point_max_y", point_max_y_, 3);
  pnh_.param<bool>("visualization", visualization_, true);
  point_min_x_ = max_distance_to_intersection_;
  ROS_INFO("Intersection server: active");
  if (visualization_)
  {
    visualize_intersection_ = nh_.advertise<visualization_msgs::Marker>("/intersection", 10);
  }
}

void IntersectionServer::init()
{
  goal_ = *(intersectionServer_.acceptNewGoal());
  intersection_subscriber_ = nh_.subscribe("/intersection", 1, &IntersectionServer::intersection_callback, this);
  obstacles_sub_ = nh_.subscribe("/obstacles", 1, &IntersectionServer::manager, this);
  speed_publisher_ = nh_.advertise<std_msgs::Float64>("/max_speed", 2);
  speed_publisher_.publish(speed_);
  publishFeedback(STOPPED_ON_INTERSECTION);
  ROS_INFO("Initialized");
}

void IntersectionServer::manager(const selfie_msgs::PolygonArray &boxes)
{
  if (!intersectionServer_.isActive())
  {
    ROS_INFO_ONCE("Search server not active");
    return;
  }
  filter_boxes(boxes);
  if (filtered_boxes_.size() != 0)
  {
    ROS_INFO_THROTTLE(1.5, "Another car on the road");
    if (visualization_)
      Box().visualizeList(filtered_boxes_, visualize_intersection_, "obstacles_on_road", 0.9, 0.9, 0.9);
    if (action_status_.action_status != FOUND_OBSTACLES)
    {
      if (max_distance_to_intersection_ > point_min_x_)
      {
        speed_publisher_.publish(speed_);
        publishFeedback(FOUND_OBSTACLES);
      } else
      {
        publishFeedback(APPROACHING_TO_INTERSECTION_WITH_OBSTACLES);
      }
    }
  } else
  {
    publishFeedback(ROAD_CLEAR);
    ROS_INFO("Road clear");
    send_goal();
  }
}

void IntersectionServer::intersection_callback(const std_msgs::Float32 &msg)
{
  point_min_x_ = msg.data;
  point_max_x_ = point_min_x_ + road_width_;
  ROS_INFO_THROTTLE(1, "Distance to intersection: %lf", point_min_x_);
}

void IntersectionServer::send_goal()
{
  selfie_msgs::intersectionResult result;
  result.done = true;
  intersectionServer_.setSucceeded();
}

void IntersectionServer::filter_boxes(const selfie_msgs::PolygonArray &msg)
{
  filtered_boxes_.clear();
  bool box_ok;
  if (!msg.polygons.empty())
  {
    for (int box_nr = msg.polygons.size() - 1; box_nr >= 0; box_nr--)
    {
      geometry_msgs::Polygon polygon = msg.polygons[box_nr];
      box_ok = true;
      for (int a = 0; a < 4; ++a)
      {
        Point p(polygon.points[a]);
        if (!p.check_position(point_min_x_, point_max_x_, point_min_y_, point_max_y_))
        {
          box_ok = false;
          break;
        }
      }
      if (box_ok)
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

void IntersectionServer::publishFeedback(program_states newStatus)
{
  action_status_.action_status = newStatus;
  intersectionServer_.publishFeedback(action_status_);
}