#include <selfie_intersection/IntersectionServer.h>

IntersectionServer::IntersectionServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh)
    , pnh_(pnh)
    , intersectionServer_(nh_, "intersection", false)
{
  intersectionServer_.registerGoalCallback(boost::bind(&IntersectionServer::init, this));
  intersectionServer_.start();
  pnh_.param<float>("point_min_x", point_min_x_, 0.01);
  pnh_.param<float>("point_max_x", point_max_x_, 0.95);
  pnh_.param<float>("point_min_y", point_min_y_, -3);
  pnh_.param<float>("point_max_y", point_max_y_, 3);
}

void IntersectionServer::init()
{
  obstacles_sub_ = nh_.subscribe("/obstacles", 1, &IntersectionServer::manager, this);
  speed_publisher_ = nh_.advertise<std_msgs::Float64>("/max_speed", 0);
}

void IntersectionServer::manager(const selfie_msgs::PolygonArray &boxes) {}

void IntersectionServer::filter_boxes(const selfie_msgs::PolygonArray &msg)
{
  filtered_boxes_.clear();
  bool box_ok;
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
      filtered_boxes_.insert(filtered_boxes_.begin(), temp_box);
    }
  }
}
// TODO filtering using also size of box