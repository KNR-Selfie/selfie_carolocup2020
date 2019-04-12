#include<iostream>

#include <geometry_msgs/Polygon.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <selfie_msgs/PolygonArray.h>

using namespace std;

class Point
{
public:

    float x;
    float y;

    Point():x(0), y(0){}
    Point(float x_, float y_):x(x_), y(y_){}
    Point(geometry_msgs::Point32 point)
    {
      x = point.x;
      y = point.y;
    }
    Point(geometry_msgs::Point point)
    {
      x = point.x;
      y = point.y;
    }
    ~Point(){}
    Point operator=(const geometry_msgs::Point32 other)
    {
      this->x = other.x;
      this->y = other.y;
      return *this;
    }
    Point operator=(const geometry_msgs::Point other)
    {
      this->x = other.x;
      this->y = other.y;
      return *this;
    } 

    void reset()
    {
      x = 0;
      y = 0;
    }

    bool check_position(float min_x, float max_x, float min_y, float max_y)
    {
    //  ROS_INFO("point limits: %f,    %f,    %f,   %f", min_x, max_x, min_y, max_y);
      if(x < min_x || x > max_x || y < min_y || y >max_y)
        return 0;
      else
        return 1;
    }
    float get_distance(const Point other)
    {
      return ( std::sqrt(std::pow(other.x - x, 2) + pow(other.y - y, 2)));
    }
    float get_distance(geometry_msgs::Point32 other)
    {
      return ( std::sqrt(std::pow(other.x - x, 2) + pow(other.y - y, 2)));
    }
    void print()
    {
      ROS_INFO("(x,y) = ( %f, %f )", x, y);
    }
};  

// y= ax+b
struct Line
{
  float a;
  float b;
};

// wspolrzedne lokalne
class Box
{
public:
    Point bottom_left;
    Point top_left;
    Point top_right;
    Point bottom_right;
    Line left_vertical_line;
    Line bottom_horizontal_line;
public:
    Box(){}
    Box(const Box &other)
    {
      bottom_left = other.bottom_left;
      bottom_right = other.bottom_right;
      top_left = other.top_left;
      top_right = other.top_right;
      left_vertical_line = other.left_vertical_line;
      bottom_horizontal_line = other.bottom_horizontal_line;
    }
    // this constructor finds and assign correct points to corners of our box
    Box(geometry_msgs::Polygon poly)
    {
      bool is_in_the_front = true;
      float avg_x = (poly.points[0].x + poly.points[1].x + poly.points[2].x + poly.points[3].x )/4;
      if(avg_x < 0)
        is_in_the_front = false;

      //point (0,0)
      Point zero(0,0);
      vector<float> distances = {zero.get_distance(poly.points[0]), zero.get_distance(poly.points[1]),
                              zero.get_distance(poly.points[2]), zero.get_distance(poly.points[3])};

      vector<float>::iterator min;

//        bottom_left = poly.points[std::distance(distances.begin(), min)];

      min = min_element(distances.begin(), distances.end());
      if(is_in_the_front)
        bottom_left = poly.points[std::distance(distances.begin(), min)];
      else  
        top_left = poly.points[std::distance(distances.begin(), min)];
      *min = 1000;
    //    cout <<"distance from the nearest point of the box: " <<  bottom_left.x << endl;

        //pick two closest points
      Point a, b;
      min = min_element(distances.begin(), distances.end());
        a = poly.points[std::distance(distances.begin(), min)];
      *min = 1000;
      min = min_element(distances.begin(), distances.end());
      b = poly.points[std::distance(distances.begin(), min)];
      *min = 1000;
      //pick the one with smaller y coordinate -> it's the left one
      if(a.x > b.x)
        {
          if(is_in_the_front)
          {
            top_left = a;
            bottom_right = b;
          }else
          {
            bottom_left = a;
            top_right = b;
          }
        }else{
          if(is_in_the_front)
          {
            top_left = b;
            bottom_right = a;
          }else
          {
            bottom_left = b;
            top_right = a;
          }
        }
      min = min_element(distances.begin(), distances.end());
      if(is_in_the_front)
        top_right = poly.points[std::distance(distances.begin(), min)];
      else
        bottom_right = poly.points[std::distance(distances.begin(), min)];
      *min = 1000;


    }

      Box(Point b_l, Point b_r, Point t_l, Point t_r):
                  bottom_left(b_l), bottom_right(b_r),
                  top_left(t_l), top_right(t_r)
      {
        make_lines();
      }
      void reset()
      {
        bottom_left.reset();
        top_left.reset();
        top_right.reset();
        bottom_right.reset();
        left_vertical_line.a = 0;
        left_vertical_line.b = 0;
        bottom_horizontal_line.a = 0;
        bottom_horizontal_line.b = 0;
      }
      
      void make_poly(geometry_msgs::Polygon &poly)
      {
       geometry_msgs::Point32 top_l;
       top_l.x = top_left.x;
       top_l.y = top_left.y;
       top_l.z = 0;
       geometry_msgs::Point32 top_r;
       top_r.x = top_right.x;
       top_r.y = top_right.y;
       top_r.z = 0;
       geometry_msgs::Point32 bottom_r;
       bottom_r.x = bottom_right.x;
       bottom_r.y = bottom_right.y;
       bottom_r.z = 0;
       geometry_msgs::Point32 bottom_l;
       bottom_l.x = bottom_left.x;
       bottom_l.y = bottom_left.y;
       bottom_l.z = 0;

        poly.points = {top_l, bottom_l, bottom_r, top_r};
      }

      void make_lines()
      {
        left_vertical_line.a = (top_left.y - bottom_left.y)/(top_left.x - bottom_left.x);
        left_vertical_line.b = top_left.y - left_vertical_line.a*top_left.x;

        bottom_horizontal_line.a = (bottom_right.y - bottom_left.y)/(bottom_right.x - bottom_left.x);
        bottom_horizontal_line.b = bottom_right.y - bottom_horizontal_line.a*bottom_right.x;
      }
      void print_lines()
      {

      }

      void visualize(ros::Publisher &pub)
      {
        Point to_draw_1 = bottom_left;
        Point to_draw_2 = top_left;
        visualization_msgs::Marker marker;

        marker.header.frame_id = "laser";
        marker.header.stamp = ros::Time::now();
        marker.ns = "box";
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 4;
        marker.lifetime = ros::Duration();

        marker.color.r = 0.4f;
        marker.color.g = 0.3f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.1;

        geometry_msgs::Point marker_point;
        marker_point.x = to_draw_1.x;
        marker_point.y = to_draw_1.y;
        marker_point.z = 0;
        marker.points.push_back(marker_point);

        marker_point.x = to_draw_1.x;
        marker_point.y = to_draw_1.y;
        marker_point.z = 0.5;
        marker.points.push_back(marker_point);

        marker_point.x = to_draw_2.x;
        marker_point.y = to_draw_2.y;
        marker_point.z = 0;
        marker.points.push_back(marker_point);

        marker_point.x = to_draw_2.x;
        marker_point.y = to_draw_2.y;
        marker_point.z = 0.5;
        marker.points.push_back(marker_point);       

        pub.publish(marker);

      }

      void print()
      {
        ROS_INFO("  bottom left: ");
        bottom_left.print();
        ROS_INFO("  bottom_right: ");
        bottom_right.print();
        ROS_INFO("  top left: ");
        top_left.print();
        ROS_INFO("  top right: ");
        top_right.print();
      }
      void print_box_dimensions()
      {
        float bottom_edge = bottom_left.get_distance(bottom_right);
        float top_edge = top_left.get_distance(top_right);
        float right_edge = top_right.get_distance(bottom_right);
        float left_edge = top_left.get_distance(bottom_left);

        ROS_INFO("bottom_edge: %f", bottom_edge);
        ROS_INFO("top_edge: %f", top_edge);
        ROS_INFO("right_edge:  %f", right_edge);
        ROS_INFO("left_edge: %f", left_edge);
      }
};