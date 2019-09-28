/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include <selfie_avoiding_obstacles/road_obstacle_detector.h>

Road_obstacle_detector::Road_obstacle_detector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh)
    , pnh_(pnh)
{
  pnh_.param<bool>("vizualization", visualization_, false);
  obstacles_sub_ = nh_.subscribe("/obstacles", 1, &Road_obstacle_detector::obstacle_callback, this);
  markings_sub_ = nh_.subscribe("/road_markings", 1, &Road_obstacle_detector::road_markings_callback, this);
  if (visualization_)
  {
    visualizer_ = nh_.advertise<visualization_msgs::Marker>("/avoiding_obstacles", 1);
  }
  received_road_markings_ = false;
  status_ = CLEAR;
  ROS_INFO("Initialized");
}

Road_obstacle_detector::~Road_obstacle_detector() {}

void Road_obstacle_detector::obstacle_callback(const selfie_msgs::PolygonArray &msg) {}

void Road_obstacle_detector::filter_boxes(const selfie_msgs::PolygonArray &msg)
{
  filtered_boxes_.clear();
  boxes_in_front_of_car_ = 0;
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
        boxes_in_front_of_car_++;
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