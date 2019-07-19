#include <selfie_intersection/IntersectionServer.h>

IntersectionServer::IntersectionServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh)
    , pnh_(pnh)
    , intersectionServer_(nh_, "intersection", false)
{
}