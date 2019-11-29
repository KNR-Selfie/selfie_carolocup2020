#include "selfie_starting_procedure/gate_scanner.h"

GateScanner::GateScanner(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
nh_(nh),pnh_(pnh)
{
    pnh_.param<float>("no_obstacle_time_thresh",noObstacleTimeThresh_,1.5f);
    pnh_.param<float>("min_distance",minDistance_,0.1);
    pnh_.param<float>("max_distance",maxDistance_,0.5);
    pnh_.param<float>("min_width",minWidth_,0.4);
    timer_ = nh_.createTimer(ros::Duration(noObstacleTimeThresh_) ,&GateScanner::timerCallback, this, true, false);
    startServ_ = nh_.advertiseService("startGateScan", &GateScanner::startSearching, this);
    gateOpenPub_ = nh_.advertise<std_msgs::Empty>("scan_gate_open",1);

}
void GateScanner::laserScanCB(const sensor_msgs::LaserScan &msg)
{
    float angle = msg.angle_min;
    for(std::vector<float>::const_iterator it = msg.ranges.begin(); it<msg.ranges.end(); ++it, angle += msg.angle_increment)
    {
        if(*it > msg.range_max || *it < msg.range_min) continue;
        Point point(angle,*it);
        {
            if(point.x > minDistance_ && point.x < maxDistance_ && std::abs(point.y) < minWidth_/2.f)
            {
                resetTimer();
                return;
            }
        }
    }
}

void GateScanner::timerCallback(const ros::TimerEvent &e)
{
    gateOpenPub_.publish(std_msgs::Empty());
    scanSub_.shutdown();
}

void GateScanner::resetTimer()
{
    timer_.setPeriod(ros::Duration(noObstacleTimeThresh_), true);//reset
    std::cout<<"timer reset"<<std::endl;
}

bool GateScanner::startSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp)
{
    scanSub_ = nh_.subscribe("scan",10, &GateScanner::laserScanCB, this);
    timer_.start();
    return true;
}




GateScanner::Point::Point(float ang, float range):x(cos(ang)*range),y(sin(ang)*range){}