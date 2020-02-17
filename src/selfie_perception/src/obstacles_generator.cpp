#include <selfie_perception/obstacles_generator.h>

ObstaclesGenerator::ObstaclesGenerator(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
  nh_(nh),
  pnh_(pnh),
  transformListener_(nh_),
  max_range_(1.0),
  min_range_(0.03),

  obstacles_frame_("laser"),
  visualization_frame_("base_link"),
  output_frame_("base_link"),

  visualize_(true),

  lidar_offset_(0),
  segment_threshold_(0.03),
  min_segment_size_(0.04),
  max_segment_size_(0.5),
  min_to_divide_(0.03),
  upside_down_(false),
  dr_server_CB_(boost::bind(&ObstaclesGenerator::reconfigureCB, this, _1, _2))
{
  obstacles_pub_ = nh_.advertise<selfie_msgs::PolygonArray>("obstacles", 10);
}

ObstaclesGenerator::~ObstaclesGenerator()
{
  obstacle_array_.polygons.clear();
  line_array_.clear();
}

bool ObstaclesGenerator::init()
{
  scan_sub_ = nh_.subscribe("/scan", 10, &ObstaclesGenerator::laserScanCallback, this);
  pnh_.getParam("max_range", max_range_);
  pnh_.getParam("min_range", min_range_);

  pnh_.getParam("visualize", visualize_);
  pnh_.getParam("obstacles_frame", obstacles_frame_);
  pnh_.getParam("visualization_frame", visualization_frame_);
  pnh_.getParam("output_frame", output_frame_);

  pnh_.getParam("segment_threshold", segment_threshold_);
  pnh_.getParam("min_segment_size", min_segment_size_);
  pnh_.getParam("max_segment_size", max_segment_size_);
  pnh_.getParam("min_to_divide", min_to_divide_);

  pnh_.getParam("lidar_offset", lidar_offset_);
  pnh_.getParam("upside_down", upside_down_);

  dr_server_.setCallback(dr_server_CB_);

  if (visualize_)
  {
    visualization_lines_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_lines", 1);
    visualization_obstacles_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_obstacles", 1);
  }

  printInfoParams();
  initializeTransform();
  return true;
}

void ObstaclesGenerator::laserScanCallback(const sensor_msgs::LaserScan &msg)
{
  scan = msg;
  divideIntoSegments();
  if(segments_.empty())
  {
    obstacle_array_.polygons.clear();
    obstacle_array_.header.stamp = ros::Time::now();
    obstacle_array_.header.frame_id = output_frame_;
    obstacles_pub_.publish(obstacle_array_);
    line_array_.clear();
  }
  else
  {
    generateLines();
    generateObstacles();
  }
  if (visualize_)
  {
    visualizeLines();
    visualizeObstacles();
  }
}

void ObstaclesGenerator::generateLines()
{
  line_array_.clear();
  for(int i = 0; i < segments_.size(); ++i)
  {
    float max = 0;
    Point p_max;
    float A = getA(segments_[i][0], segments_[i][segments_[i].size() - 1]);
    float C = segments_[i][0].y - (A * segments_[i][0].x);
    for(int j = 0; j < segments_[i].size(); ++j)
    {
      float distance = std::abs(A * segments_[i][j].x - segments_[i][j].y + C) / sqrtf(A * A + 1);
      if(distance > max)
      {
        max = distance;
        p_max = segments_[i][j];
      }
    }
    if(max > min_to_divide_)
    {
      Line l;
      l.start_point = segments_[i][0];
      l.end_point = p_max;
      l.slope = getSlope(l.start_point, l.end_point);
      l.a = getA(l.start_point, l.end_point);
      l.b = l.start_point.y - (l.a * l.start_point.x);
      line_array_.push_back(l);
      l.start_point = p_max;
      l.end_point = segments_[i][segments_[i].size() - 1];
      l.slope = getSlope(l.start_point, l.end_point);
      l.a = getA(l.start_point, l.end_point);
      l.b = l.start_point.y - (l.a * l.start_point.x);
      line_array_.push_back(l);
    }
    else
    {
      Line l;
      l.start_point = segments_[i][0];
      l.end_point = segments_[i][segments_[i].size() - 1];
      l.slope = getSlope(l.start_point, l.end_point);
      l.a = getA(l.start_point, l.end_point);
      l.b = l.start_point.y - (l.a * l.start_point.x);
      line_array_.push_back(l);
    }

  }
}

Point ObstaclesGenerator::getXY(float angle, float range)
{
  Point p;
  p.x = range * cos(angle);
  p.y = range * sin(angle);
  return p;
}

float ObstaclesGenerator::getSlope(Point &p1, Point &p2)
{
  return atan((p2.y - p1.y) / (p2.x - p1.x));
}

float ObstaclesGenerator::getDistance(Point &p1, Point &p2)
{
  float dx = p2.x - p1.x;
  float dy = p2.y - p1.y;
  return std::sqrt(dx * dx + dy * dy);
}

float ObstaclesGenerator::getA(Point &p1, Point &p2)
{
  return (p2.y - p1.y) / (p2.x - p1.x);
}

void ObstaclesGenerator::visualizeLines()
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = visualization_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "line";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.lifetime = ros::Duration();

  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;

  geometry_msgs::Point marker_point;
  marker_point.z = 0;
  if (!line_array_.empty())
  {
    for (int i = 0; i < line_array_.size(); i++)
    {
      marker_point.x = line_array_[i].start_point.x + lidar_offset_;
      marker_point.y = line_array_[i].start_point.y * 1;
      marker.points.push_back(marker_point);

      marker_point.x = line_array_[i].end_point.x + lidar_offset_;
      marker_point.y = line_array_[i].end_point.y * 1;
      marker.points.push_back(marker_point);
    }
  }
  visualization_lines_pub_.publish(marker);
}

void ObstaclesGenerator::printInfoParams()
{
  ROS_INFO("max_range: %.3f", max_range_);
  ROS_INFO("min_range: %.3f\n", min_range_);

  ROS_INFO("visualize: %d", visualize_);
  ROS_INFO("obstacles_frame: %s", obstacles_frame_.c_str());
  ROS_INFO("visualization_frame: %s\n", visualization_frame_.c_str());

  ROS_INFO("segment_threshold: %.3f", segment_threshold_);
  ROS_INFO("min_segment_size: %.3f", min_segment_size_);
  ROS_INFO("max_segment_size: %.3f", max_segment_size_);
  ROS_INFO("min_to_divide: %.3f\n", min_to_divide_);

  ROS_INFO("lidar_offset: %.3f", lidar_offset_);
  ROS_INFO("upside_down: %d", upside_down_);
}

void ObstaclesGenerator::generateObstacles()
{
  obstacle_array_.polygons.clear();
  obstacle_array_.header.stamp = ros::Time::now();
  obstacle_array_.header.frame_id = output_frame_;
  if (!line_array_.empty())
  {
    float distance = 0;
    float max_distance = 0.16;
    float slope_diff = 0;
    geometry_msgs::Point32 p;
    p.z = 0;
    geometry_msgs::Polygon obstacle;
    bool obstacle_generated = false;
    for (int i = 0; i < line_array_.size(); i++)
    {
      obstacle_generated = false;
      if (i != line_array_.size() - 1)
      {
        distance = getDistance(line_array_[i].end_point, line_array_[i + 1].start_point);
        slope_diff = std::abs(line_array_[i].slope - line_array_[i + 1].slope);
        if (distance < max_distance && slope_diff > M_PI / 4.2)
        {
          p.x = ((line_array_[i + 1].b - line_array_[i].b) / (line_array_[i].a - line_array_[i + 1].a)) + lidar_offset_;
          p.y = (((line_array_[i + 1].b * line_array_[i].a) - (line_array_[i].b * line_array_[i + 1].a)) / (line_array_[i].a - line_array_[i + 1].a));
          obstacle.points.push_back(p);
          p.x = line_array_[i].start_point.x + lidar_offset_;
          p.y = line_array_[i].start_point.y;
          obstacle.points.push_back(p);
          float b1 = line_array_[i].start_point.y - line_array_[i + 1].a * line_array_[i].start_point.x;
          float b2 = line_array_[i + 1].end_point.y - line_array_[i].a * line_array_[i + 1].end_point.x;
          p.x = ((b1 - b2) / (line_array_[i].a - line_array_[i + 1].a)) + lidar_offset_;
          p.y = ((b1 * line_array_[i].a - b2 * line_array_[i + 1].a) / (line_array_[i].a - line_array_[i + 1].a));
          obstacle.points.push_back(p);
          p.x = line_array_[i + 1].end_point.x + lidar_offset_;
          p.y = line_array_[i + 1].end_point.y;
          obstacle.points.push_back(p);
          i++;
          obstacle_generated = true;
        }
      }
      if (!obstacle_generated)
      {
        float obstacle_nominal_length_ = getDistance(line_array_[i].start_point, line_array_[i].end_point);
        p.x = line_array_[i].start_point.x + lidar_offset_;
        p.y = line_array_[i].start_point.y;
        obstacle.points.push_back(p);

        p.x = line_array_[i].end_point.x + lidar_offset_;
        p.y = line_array_[i].end_point.y;
        obstacle.points.push_back(p);

        float add_x = obstacle_nominal_length_ * sin(line_array_[i].slope);
        float add_y = obstacle_nominal_length_ * cos(line_array_[i].slope);

        float sx = (line_array_[i].end_point.x + line_array_[i].start_point.x) / 2;
        float sy = (line_array_[i].end_point.y + line_array_[i].start_point.y) / 2;

        if (line_array_[i].slope > -1 * M_PI / 4 && line_array_[i].slope < M_PI / 4)
        {
          if (sy > 0)
          {
            add_x *= -1;
            add_y *= -1;
          }
        }
        else if ((sx > 0 && line_array_[i].slope < 0) || (sx < 0 && line_array_[i].slope > 0))
        {
          add_x *= -1;
          add_y *= -1;
        }

        p.x = line_array_[i].end_point.x + add_x + lidar_offset_;
        p.y = (line_array_[i].end_point.y - add_y);
        obstacle.points.push_back(p);

        p.x = line_array_[i].start_point.x + add_x + lidar_offset_;
        p.y = (line_array_[i].start_point.y - add_y);
        obstacle.points.push_back(p);
      }
      obstacle_array_.polygons.push_back(obstacle);
      obstacle.points.clear();
    }
  }
  convertToOutputFrame(); // obstacle_array_
  obstacles_pub_.publish(obstacle_array_);
}

void ObstaclesGenerator::visualizeObstacles()
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = visualization_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "obstacles";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.lifetime = ros::Duration();

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.scale.x = 0.006;
  marker.scale.y = 0.006;

  geometry_msgs::Point marker_point;
  marker_point.z = 0;

  if (!obstacle_array_.polygons.empty())
  {
    for (int i = 0; i < obstacle_array_.polygons.size(); i++)
    {
      marker_point.x = obstacle_array_.polygons[i].points[0].x;
      marker_point.y = obstacle_array_.polygons[i].points[0].y;
  
      marker.points.push_back(marker_point);
  
      marker_point.x = obstacle_array_.polygons[i].points[1].x;
      marker_point.y = obstacle_array_.polygons[i].points[1].y;
  
      marker.points.push_back(marker_point);
  
      marker_point.x = obstacle_array_.polygons[i].points[1].x;
      marker_point.y = obstacle_array_.polygons[i].points[1].y;
  
      marker.points.push_back(marker_point);
  
      marker_point.x = obstacle_array_.polygons[i].points[2].x;
      marker_point.y = obstacle_array_.polygons[i].points[2].y;
  
      marker.points.push_back(marker_point);
  
      marker_point.x = obstacle_array_.polygons[i].points[2].x;
      marker_point.y = obstacle_array_.polygons[i].points[2].y;
  
      marker.points.push_back(marker_point);
  
      marker_point.x = obstacle_array_.polygons[i].points[3].x;
      marker_point.y = obstacle_array_.polygons[i].points[3].y;
  
      marker.points.push_back(marker_point);
  
      marker_point.x = obstacle_array_.polygons[i].points[3].x;
      marker_point.y = obstacle_array_.polygons[i].points[3].y;
  
      marker.points.push_back(marker_point);
  
      marker_point.x = obstacle_array_.polygons[i].points[0].x;
      marker_point.y = obstacle_array_.polygons[i].points[0].y;
  
      marker.points.push_back(marker_point);
    }
  }
  visualization_obstacles_pub_.publish(marker);
}

void ObstaclesGenerator::divideIntoSegments()
{
  segments_.clear();
  std::vector <Point> segment;
  Point p = getXY(scan.angle_min, scan.ranges[0]);
  segment.push_back(p);
  int act_angle_index = 1;
  bool segment_end = false;
  for (float act_angle = scan.angle_min + scan.angle_increment; act_angle <= scan.angle_max; act_angle += scan.angle_increment)
  {
    if (scan.ranges[act_angle_index] <= max_range_ && scan.ranges[act_angle_index] >= min_range_)
    {
      if(std::abs(scan.ranges[act_angle_index] - scan.ranges[act_angle_index - 1]) < segment_threshold_)
      {
        p = getXY(act_angle, scan.ranges[act_angle_index]);
        segment.push_back(p);
      }
      else
        segment_end = true;
    }
    else
      segment_end = true;

    if(segment_end)
    {
      if(!segment.empty())
      {
        float segment_size = getDistance(segment[0], segment[segment.size() - 1]);
        if(segment_size > min_segment_size_ && segment_size < max_segment_size_)
        {
          segments_.push_back(segment);
          segment.clear();
        }
        else
          segment.clear();
      }
      segment_end = false;
    }
    ++act_angle_index;
  }
}

void ObstaclesGenerator::convertToOutputFrame()
{
  if(output_frame_ == obstacles_frame_) return;

  for(std::vector<geometry_msgs::Polygon>::iterator plgit = obstacle_array_.polygons.begin();plgit!= obstacle_array_.polygons.end();++plgit)
  {
    std::for_each(plgit->points.begin(), plgit->points.end(), std::bind(&ObstaclesGenerator::transformPoint,this,  std::placeholders::_1)); 
  }
}

void ObstaclesGenerator::initializeTransform()
{
  ROS_INFO("Waiting for any transform form %s to %s\n",output_frame_.c_str(), obstacles_frame_.c_str());

  transformListener_.waitForTransform(output_frame_, obstacles_frame_, ros::Time(0), ros::Duration(30), ros::Duration(0.0001));
  ROS_INFO("Transform from %s to %s found\n",output_frame_.c_str(), obstacles_frame_.c_str());
  transformListener_.lookupTransform(output_frame_, obstacles_frame_, ros::Time(0), transform_);
}

void ObstaclesGenerator::transformPoint(geometry_msgs::Point32 &pt32)
{
  tf::Point tfpt;
  geometry_msgs::Point pt;
  pt.x = static_cast<double>(pt32.x);
  pt.y = static_cast<double>(pt32.y);
  pt.z = static_cast<double>(pt32.z);
  tf::pointMsgToTF(pt, tfpt);
  tfpt = transform_ * tfpt;
  tf::pointTFToMsg(tfpt, pt);
  pt32.x = static_cast<float>(pt.x);
  pt32.y = static_cast<float>(pt.y);
  pt32.z = static_cast<float>(pt.z);

}

void ObstaclesGenerator::convertUpsideDown()
{
  for(int i = 0; i < obstacle_array_.polygons.size(); ++i)
  {
    obstacle_array_.polygons[i].points[0].y *= -1;
    obstacle_array_.polygons[i].points[1].y *= -1;
    obstacle_array_.polygons[i].points[2].y *= -1;
    obstacle_array_.polygons[i].points[3].y *= -1;
  }
}
void ObstaclesGenerator::reconfigureCB(selfie_perception::DetectObstaclesConfig& config, uint32_t level)
{
    if(max_range_ != (float)config.max_range)
    {
        max_range_ = config.max_range;
        ROS_INFO("max_range_ new value: %f", max_range_);
    }
    if(max_segment_size_ != (float)config.max_segment_size)
    {
        max_segment_size_ = config.max_segment_size;
        ROS_INFO("max_segment_size_ new value: %f", max_segment_size_);
    }
    if(min_range_ != (float)config.min_range)
    {
        min_range_ = config.min_range;
        ROS_INFO("min_range_ new value: %f", min_range_);
    }
    if(min_segment_size_ != (float)config.min_segment_size)
    {
        min_segment_size_ = config.min_segment_size;
        ROS_INFO("min_segment_size_ new value: %f", min_segment_size_);
    }
    if(min_to_divide_ != (float)config.min_to_divide)
    {
        min_to_divide_ = config.min_to_divide;
        ROS_INFO("min_to_divide new value: %f", min_to_divide_);
    }
    if(segment_threshold_ != (float)config.segment_threshold)
    {
        segment_threshold_ = config.segment_threshold;
        ROS_INFO("segment_threshold new value: %f", segment_threshold_);
    }

}
