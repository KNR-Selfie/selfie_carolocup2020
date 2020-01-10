#pragma once
#ifndef PACKAGE_PATH_FILE_H
#define PACKAGE_PATH_FILE_H

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <functional>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/Marker.h>
#include <selfie_msgs/PolygonArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <selfie_perception/DetectObstaclesConfig.h>

struct Point
{
    float x;
    float y;
};

struct Line
{
    Point start_point;
    Point end_point;
    float slope;
    float b;
    float a;
    float length;
};

class ObstaclesGenerator
{
public:
    ObstaclesGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~ObstaclesGenerator();
    bool init();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber scan_sub_;
    ros::Publisher obstacles_pub_;
    ros::Publisher visualization_lines_pub_;
    ros::Publisher visualization_obstacles_pub_;
    tf::TransformListener transformListener_;

    tf::StampedTransform transform_;

    dynamic_reconfigure::Server<selfie_perception::DetectObstaclesConfig> dr_server_;
    dynamic_reconfigure::Server<selfie_perception::DetectObstaclesConfig>::CallbackType dr_server_CB_;
    void reconfigureCB(selfie_perception::DetectObstaclesConfig& config, uint32_t level);

    std::vector <Line> line_array_;
    std::vector <std::vector <Point> > segments_;
    selfie_msgs::PolygonArray obstacle_array_;
    sensor_msgs::LaserScan scan;
    sensor_msgs::LaserScan scan_;
    void laserScanCallback(const sensor_msgs::LaserScan& msg);
    void divideIntoSegments();
    void generateLines();
    Point getXY(float angle, float range);
    float getSlope(Point &p1, Point &p2);
    float getA(Point &p1, Point &p2);
    float getDistance(Point &p1, Point &p2);
    void visualizeLines();
    void visualizeObstacles();
    void printInfoParams();
    void generateObstacles();
    void convertUpsideDown();
    void convertToOutputFrame();
    void initializeTransform();
    void transformPoint(geometry_msgs::Point32 &);

    float max_range_;
    float min_range_;
    bool visualize_;
    bool upside_down_;
    float lidar_offset_;
    std::string visualization_frame_;
    std::string obstacles_frame_;
    std::string output_frame_;
    float segment_threshold_;
    float min_segment_size_;
    float max_segment_size_;
    float min_to_divide_;
};

    
#endif
