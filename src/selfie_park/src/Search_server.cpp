#include "../../selfie_park/include/selfie_park/Search_server.h"


/*TODO LIST:
planning_failed
check //TEMP //TODO

check OPT -optymalizacja
*/

Search_server::Search_server(const ros::NodeHandle &nh, const ros::NodeHandle &pnh):
                nh_(nh),
                pnh_(pnh),
                search_server_(nh_, "search",  false)
{
  search_server_.registerGoalCallback(boost::bind(&Search_server::init, this)); 
//  search_server_.registerPreemptCallback(boost::bind(&Search_server::preemptCB, this)); TODO
// TODO Result CB
  search_server_.start();
  pnh_.param<float>("point_min_x", point_min_x, 0.01);
  pnh_.param<float>("point_max_x", point_max_x, 2);
  pnh_.param<float>("point_min_y", point_min_y, -1);
  pnh_.param<float>("point_max_y", point_max_y, 0.2);
  pnh_.param<float>("distance_to_stop", distance_to_stop, 0.2);
  pnh_.param<int>("scans_to_ignore_when_stopped", scans_ignored, 2);
  pnh_.param<int>("scans_taken", scans_taken, 5);
  pnh_.param<bool>("debug_mode", debug_mode, false);
  pnh_.param<int>("visualization_type", visualization_type, 3);
}

Search_server::~Search_server(){}

bool Search_server::init()
{
  this->obstacles_sub = nh_.subscribe("/obstacles", 10, &Search_server::manager, this);
  this->visualize_lines_pub = nh_.advertise<visualization_msgs::Marker>( "/visualization_lines", 1 );
  this->visualize_free_place  = nh_.advertise<visualization_msgs::Marker>( "/free_place", 1 );
  this->point_pub = nh_.advertise<visualization_msgs::Marker>("/box_points", 5);

  min_spot_lenght = (*search_server_.acceptNewGoal()).min_spot_lenght;
  ROS_INFO("Initialized");
}


void Search_server::manager(const selfie_msgs::PolygonArray &msg)
{ 
  // to save cpu time just do nothing when new scan comes
 if(!search_server_.isActive())
  {
    ROS_INFO("search_server_ is not active");
    return;
  }
  filter_boxes(msg);
  display_places(boxes_on_the_right_side,"FilteredBoxes");
  //ROS_INFO("Size of  boxes_on_the_right %lu",boxes_on_the_right_side.size());
  if(!find_free_places()){
    ROS_INFO("Didn't found free places\n");
  }else
    send_goal();
 
}

void Search_server::filter_boxes(const selfie_msgs::PolygonArray &msg)
{
  this->boxes_on_the_right_side.clear();
  for(int box_nr = msg.polygons.size()-1;  box_nr >= 0;  box_nr--)
  {
    float min_x = point_min_x;
    float max_x = point_max_x;
    float min_y = point_min_y;
    float max_y = point_max_y;

    geometry_msgs::Polygon polygon = msg.polygons[box_nr];
    bool box_ok = true;
    for(int a = 0;  a < 4;  ++a)
    {
      Point p(polygon.points[a]);
      if( !p.check_position(min_x, max_x, min_y, max_y ))
      {
        box_ok = false;
        break;
      }
    }
    if(box_ok)
    {
     Box temp_box(polygon);
     min_x = temp_box.top_left.x;
     this->boxes_on_the_right_side.insert(this->boxes_on_the_right_side.begin(),temp_box);
     temp_box.print();
    }
      //now we reset, but later
  }//box_nr for

}//obstacle_callback



bool Search_server::find_free_places()///TODO naprawić kolejność wykrywania miejsca obecnie freeplace oddaje najdalsze miejsce a nie najbliższe
{
  if(boxes_on_the_right_side.size() < 2)
  {
   // ROS_WARN("REJECTED");
    return false;

  }
  float min_space = min_spot_lenght;
  //ROS_INFO("min space: %f", min_space);
  vector<Box>::iterator iter = boxes_on_the_right_side.begin();
  vector<Box>::const_iterator end_iter = boxes_on_the_right_side.cend();
  for(;  iter+1 != end_iter;  ++iter)
  {
    double dist = (*iter).top_left.get_distance((*(iter+1)).bottom_left);
    ROS_INFO("dist: %f", dist);
    if(dist > min_space)
    {
      
      Box tmp_box((*iter).top_left, (*iter).top_right, (*(iter+1)).bottom_left, (*(iter+1)).bottom_right);
      first_free_place = tmp_box;
      ROS_INFO("Found place \nTL: x=%lf y=%lf\nTR: x=%lf y=%lf\nBL x=%lf y=%lf\nBR x=%lf y=%lf\n",tmp_box.top_left.x,tmp_box.top_left.y,tmp_box.top_right.x,tmp_box.top_right.y,tmp_box.bottom_left.x,tmp_box.bottom_left.y,tmp_box.bottom_right.x,tmp_box.bottom_right.y);
      display_place(tmp_box,"first_free_place");
      return true;
    }
  }
  return false;
}


void Search_server::send_goal()
{

    geometry_msgs::Point32 p;
    p.x = first_free_place.bottom_left.x;
    p.y = first_free_place.bottom_left.y;
    result.parking_spot.points.push_back(p);
    p.x = first_free_place.bottom_right.x;
    p.y = first_free_place.bottom_right.y;
    result.parking_spot.points.push_back(p);
    p.x = first_free_place.top_right.x;
    p.y = first_free_place.top_right.y;
    result.parking_spot.points.push_back(p);
    p.x = first_free_place.top_left.x;
    p.y = first_free_place.top_left.y;
    result.parking_spot.points.push_back(p);

    //reset(); TODO
    ROS_INFO("Place found and sent");
    search_server_.setSucceeded(result);
}


void Search_server::display_place(Box &place, const std::string &name)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.lifetime = ros::Duration();

    marker.color.r = 100.0f;
    marker.color.g = 255.0f;
    marker.color.b = 200.0f;
    marker.color.a = 1.0f;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;

    geometry_msgs::Point marker_point;
    marker_point.z = 0;

          marker_point.x = place.bottom_left.x;
          marker_point.y = place.bottom_left.y;
          marker.points.push_back(marker_point);
          marker_point.x = place.bottom_right.x;
          marker_point.y = place.bottom_right.y;
          marker.points.push_back(marker_point);

          marker_point.x = place.bottom_right.x;
          marker_point.y = place.bottom_right.y;
          marker.points.push_back(marker_point);
          marker_point.x = place.top_right.x;
          marker_point.y = place.top_right.y;
          marker.points.push_back(marker_point);

          marker_point.x = place.top_right.x;
          marker_point.y = place.top_right.y;
          marker.points.push_back(marker_point);
          marker_point.x = place.top_left.x;
          marker_point.y = place.top_left.y;
          marker.points.push_back(marker_point);

          marker_point.x = place.top_left.x;
          marker_point.y = place.top_left.y;
          marker.points.push_back(marker_point);
          marker_point.x = place.bottom_left.x;
          marker_point.y = place.bottom_left.y;
          marker.points.push_back(marker_point);

          visualize_free_place.publish(marker);
}



void Search_server::display_places(std::vector<Box> &boxes, const std::string &name)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.lifetime = ros::Duration();

    marker.color.r = 0.0f;
    marker.color.g = 255.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;

    geometry_msgs::Point marker_point;
    marker_point.z = 0;
    for(int i=boxes.size()-1;i>=0;i--){
      marker_point.x =boxes[i].bottom_left.x;
      marker_point.y =boxes[i].bottom_left.y;
      marker.points.push_back(marker_point);
      marker_point.x =boxes[i].bottom_right.x;
      marker_point.y =boxes[i].bottom_right.y;
      marker.points.push_back(marker_point);

      marker_point.x =boxes[i].bottom_right.x;
      marker_point.y =boxes[i].bottom_right.y;
      marker.points.push_back(marker_point);
      marker_point.x =boxes[i].top_right.x;
      marker_point.y =boxes[i].top_right.y;
      marker.points.push_back(marker_point);

      marker_point.x =boxes[i].top_right.x;
      marker_point.y =boxes[i].top_right.y;
      marker.points.push_back(marker_point);
      marker_point.x =boxes[i].top_left.x;
      marker_point.y =boxes[i].top_left.y;
      marker.points.push_back(marker_point);

      marker_point.x =boxes[i].top_left.x;
      marker_point.y =boxes[i].top_left.y;
      marker.points.push_back(marker_point);
      marker_point.x =boxes[i].bottom_left.x;
      marker_point.y =boxes[i].bottom_left.y;
      marker.points.push_back(marker_point);

    }
    visualize_free_place.publish(marker);
  }//OPT