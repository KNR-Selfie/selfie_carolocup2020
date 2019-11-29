#ifndef GATE_SCANNER_H 
#define GATE_SCANNER_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>

class GateScanner
{


    ros::NodeHandle nh_,pnh_;
    ros::Subscriber scanSub_; 
    ros::ServiceServer startServ_;
    ros::Publisher gateOpenPub_;



    float noObstacleTimeThresh_;
    float minWidth_;
    float minDistance_;
    float maxDistance_;

    ros::Timer timer_;

    bool startSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp);
    void laserScanCB(const sensor_msgs::LaserScan &msg);
    void timerCallback(const ros::TimerEvent &e);
    void resetTimer();

    struct Point
    {
       float x;
       float y;
       Point(float ang, float range);
    };
public:
    GateScanner(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

};
#endif