#include <selfie_intersection/IntersectionServer.h>

IntersectionServer::IntersectionServer(const ros::NodeHandle &nh,
                                       const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), intersectionServer_(nh_, "intersection", false) {
  pnh_.param<float>("point_min_x", point_min_x_, 0.01);
  pnh_.param<float>("point_max_x", point_max_x_, 0.95);
  pnh_.param<float>("point_min_y", point_min_y_, -3);
  pnh_.param<float>("point_max_y", point_max_y_, 3);
}