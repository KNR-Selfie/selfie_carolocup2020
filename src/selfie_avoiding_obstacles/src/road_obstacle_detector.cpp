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
  // obstacles_sub_ = nh_.subscribe("/obstacles", 1, &Road_obstacle_detector::obstacle_callback, this);
  // markings_sub_ = nh_.subscribe("/road_markings", 1, &Road_obstacle_detector::road_markings_callback, this);
  if (visualization_)
  {
    visualizer_ = nh_.advertise<visualization_msgs::Marker>("/avoiding_obstacles", 1);
  }

  ROS_INFO("Initialized");
}

Road_obstacle_detector::~Road_obstacle_detector() {}

void Road_obstacle_detector::filter_boxes(const selfie_msgs::PolygonArray &msg)
{
  filtered_boxes_.clear();
  geometry_msgs::Polygon polygon;
  for (int box_nr = msg.polygons.size() - 1; box_nr >= 0; box_nr--)
  {
    polygon = msg.polygons[box_nr];
    bool box_ok = true;
    for (int a = 0; a < 4; ++a)
    {
      Point p(polygon.points[a]);

      if (!is_on_right_lane(p))
      {
        box_ok = false;
        break;
      }
    }
    if (box_ok)
    {
      Box temp_box(polygon);
      filtered_boxes_.insert(filtered_boxes_.begin(), temp_box);
    }
  }
}

bool Road_obstacle_detector::is_on_right_lane(const Point &point) {}