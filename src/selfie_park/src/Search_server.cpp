 /**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/ 

#include <selfie_park/Search_server.h>


Search_server::Search_server(const ros::NodeHandle &nh,
                             const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), search_server_(nh_, "search", false) {
  search_server_.registerGoalCallback(boost::bind(&Search_server::init, this));
  search_server_.registerPreemptCallback(
      boost::bind(&Search_server::preemptCB, this));
  search_server_.start();
  pnh_.param<float>("point_min_x", point_min_x, 0.01);
  pnh_.param<float>("point_max_x", point_max_x, 2);
  pnh_.param<float>("point_min_y", point_min_y, -1);
  pnh_.param<float>("point_max_y", point_max_y, 0.2);
  pnh_.param<float>("default_speed_in_parking_zone",
                    default_speed_in_parking_zone, 0.3);
  pnh_.param<bool>("visualization_in_searching", visualization, false);

  speed_current.data = default_speed_in_parking_zone;
}

Search_server::~Search_server() {}

bool Search_server::init() {
  this->obstacles_sub =
      nh_.subscribe("/obstacles", 1, &Search_server::manager, this);
  this->visualize_lines_pub =
      nh_.advertise<visualization_msgs::Marker>("/visualization_lines", 1);
  this->visualize_free_place =
      nh_.advertise<visualization_msgs::Marker>("/free_place", 1);
  this->point_pub = nh_.advertise<visualization_msgs::Marker>("/box_points", 5);
  this->speed_publisher = nh_.advertise<std_msgs::Float64>("/max_speed", 0.5);

  speed_publisher.publish(speed_current);
  min_spot_lenght = search_server_.acceptNewGoal()->min_spot_lenght;
  publishFeedback(START_SEARCHING_PLACE);
  ROS_INFO("Initialized");
}

void Search_server::manager(const selfie_msgs::PolygonArray &msg) {
  // to save cpu time just do nothing when new scan comes
  if (!search_server_.isActive()) {
    ROS_INFO("search_server_ is not active");
    return;
  }
  filter_boxes(msg);
  if (visualization)
    display_places(boxes_on_the_right_side, "FilteredBoxes");
  // ROS_INFO("Size of  boxes_on_the_right %lu",boxes_on_the_right_side.size());

  switch (action_status.action_status) {
  case START_SEARCHING_PLACE:
    if (find_free_places()) {
      publishFeedback(FOUND_PLACE_MEASURING);
      speed_current.data = 0.5;
      speed_publisher.publish(speed_current);
      ros::Duration(0.4).sleep();//waiting, because next measure should be taken when car is stopped
    }
    break;

  case FOUND_PLACE_MEASURING:
    if (find_free_places()) {
      if (first_free_place.bottom_left.x <= 0.3)
        publishFeedback(FIND_PROPER_PLACE);
    } else 
    {
      speed_current.data = default_speed_in_parking_zone;
      publishFeedback(START_SEARCHING_PLACE);
    }
    speed_publisher.publish(speed_current);
    break;

  case FIND_PROPER_PLACE:
    if (find_free_places()) {
      std::cout << "Found proper place\nsending result";
      speed_current.data = 0.5;
      send_goal();
    } else {
      std::cout << "Place lost\n";
      speed_current.data = default_speed_in_parking_zone;
    }
    speed_publisher.publish(speed_current);
    break;

  default:
    ROS_INFO("Err, wrong action_status");
    break;
  }
}

void Search_server::filter_boxes(const selfie_msgs::PolygonArray &msg) {
  this->boxes_on_the_right_side.clear();
  for (int box_nr = msg.polygons.size() - 1; box_nr >= 0; box_nr--) {
    float min_x = point_min_x;
    float max_x = point_max_x;
    float min_y = point_min_y;
    float max_y = point_max_y;

    geometry_msgs::Polygon polygon = msg.polygons[box_nr];
    bool box_ok = true;
    for (int a = 0; a < 4; ++a) {
      Point p(polygon.points[a]);
      if (!p.check_position(min_x, max_x, min_y, max_y)) {
        box_ok = false;
        break;
      }
    }
    if (box_ok) {
      Box temp_box(polygon);
      min_x = temp_box.top_left.x;
      this->boxes_on_the_right_side.insert(
          this->boxes_on_the_right_side.begin(), temp_box);
    }
  }
}

bool Search_server::find_free_places() {
  if (boxes_on_the_right_side.size() < 2) {
    // ROS_WARN("REJECTED");
    return false;
  }
  float min_space = min_spot_lenght;
  // ROS_INFO("min space: %f", min_space);
  vector<Box>::iterator iter = boxes_on_the_right_side.begin();
  vector<Box>::const_iterator end_iter = boxes_on_the_right_side.cend();
  for (; iter + 1 != end_iter; ++iter) {
    double dist = (*iter).top_left.get_distance((*(iter + 1)).bottom_left);
    ROS_INFO("dist: %f", dist);
    if (dist > min_space) {

      Box tmp_box((*iter).top_left, (*iter).top_right,
                  (*(iter + 1)).bottom_left, (*(iter + 1)).bottom_right);
      first_free_place = tmp_box;
      ROS_INFO("Found place \nTL: x=%lf y=%lf\nTR: x=%lf y=%lf\nBL x=%lf "
               "y=%lf\nBR x=%lf y=%lf\n",
               tmp_box.top_left.x, tmp_box.top_left.y, tmp_box.top_right.x,
               tmp_box.top_right.y, tmp_box.bottom_left.x,
               tmp_box.bottom_left.y, tmp_box.bottom_right.x,
               tmp_box.bottom_right.y);
      if (visualization)
        display_place(tmp_box, "first_free_place");
      return true;
    }
  }
  return false;
}

void Search_server::send_goal() {

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

  // reset(); TODO
  ROS_INFO("Place found and sent");
  search_server_.setSucceeded(result);
}

void Search_server::display_place(Box &place, const std::string &name) {
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

void Search_server::display_places(std::vector<Box> &boxes,
                                   const std::string &name) {
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
  for (int i = boxes.size() - 1; i >= 0; i--) {
    marker_point.x = boxes[i].bottom_left.x;
    marker_point.y = boxes[i].bottom_left.y;
    marker.points.push_back(marker_point);
    marker_point.x = boxes[i].bottom_right.x;
    marker_point.y = boxes[i].bottom_right.y;
    marker.points.push_back(marker_point);

    marker_point.x = boxes[i].bottom_right.x;
    marker_point.y = boxes[i].bottom_right.y;
    marker.points.push_back(marker_point);
    marker_point.x = boxes[i].top_right.x;
    marker_point.y = boxes[i].top_right.y;
    marker.points.push_back(marker_point);

    marker_point.x = boxes[i].top_right.x;
    marker_point.y = boxes[i].top_right.y;
    marker.points.push_back(marker_point);
    marker_point.x = boxes[i].top_left.x;
    marker_point.y = boxes[i].top_left.y;
    marker.points.push_back(marker_point);

    marker_point.x = boxes[i].top_left.x;
    marker_point.y = boxes[i].top_left.y;
    marker.points.push_back(marker_point);
    marker_point.x = boxes[i].bottom_left.x;
    marker_point.y = boxes[i].bottom_left.y;
    marker.points.push_back(marker_point);
  }
  visualize_free_place.publish(marker);
} // OPT
void Search_server::publishFeedback(unsigned int newActionStatus) {
  action_status.action_status = newActionStatus;
  search_server_.publishFeedback(action_status);
}

void Search_server::preemptCB(){
  ROS_INFO("Action preempted");
  search_server_.setPreempted();
}