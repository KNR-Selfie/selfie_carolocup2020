/**
*Copyright ( c ) 2019, KNR Selfie
*This code is licensed under BSD license (see LICENSE for details)
**/

#include <selfie_park/Search_server.h>

Search_server::Search_server(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh)
    , pnh_(pnh)
    , search_server_(nh_, "search", false)
    , dr_server_CB_(boost::bind(&Search_server::reconfigureCB, this, _1, _2))
    , old_setpoint_(-0.2)
{
  search_server_.registerGoalCallback(boost::bind(&Search_server::init, this));
  search_server_.registerPreemptCallback(boost::bind(&Search_server::preemptCB, this));
  dr_server_.setCallback(dr_server_CB_);

  search_server_.start();
  pnh_.param<float>("point_min_x", point_min_x, 0.01);
  pnh_.param<float>("point_max_x", point_max_x, 2);
  pnh_.param<float>("point_min_y", point_min_y, -1);
  pnh_.param<float>("point_max_y", point_max_y, 0.2);
  pnh_.param<float>("default_speed_in_parking_zone", default_speed_in_parking_zone, 0.9);
  pnh_.param<float>("speed_when_found_place", speed_when_found_place, 0.3);
  pnh_.param<bool>("visualization_in_searching", visualization, true);
  pnh_.param<float>("max_distance_to_free_place", max_distance_to_free_place_, 0.8);
  pnh_.param<float>("box_angle_deg", tangens_of_box_angle_, 55); // maximum angle between car and found place
  pnh_.param<float>("length_of_parking_area", length_of_parking_area_, 5.5);
  pnh_.param<float>("new_setpoint", new_setpoint_, -0.3);
  pnh_.param<float>("old_setpoint", old_setpoint_, -0.2);
  tangens_of_box_angle_ = tan(tangens_of_box_angle_ * M_PI / 180);

  speed_publisher = nh_.advertise<std_msgs::Float64>("/max_speed", 5);

  speed_current.data = default_speed_in_parking_zone;
  if (visualization)
  {
    visualize_free_place = nh_.advertise<visualization_msgs::Marker>("/free_place", 1);
  }
}

Search_server::~Search_server() {}

bool Search_server::init()
{
  obstacles_sub = nh_.subscribe("/obstacles", 1, &Search_server::manager, this);
  distance_sub_ = nh_.subscribe("/distance", 1, &Search_server::distanceCb, this);

  speed_current.data = default_speed_in_parking_zone;
  speed_publisher.publish(speed_current);
  changeSetpoint(new_setpoint_);
  min_spot_lenght = search_server_.acceptNewGoal()->min_spot_lenght;
  publishFeedback(START_SEARCHING_PLACE);
  ROS_INFO("Initialized");
}

void Search_server::manager(const selfie_msgs::PolygonArray &msg)
{
  // to save cpu time just do nothing when new scan comes
  if (!search_server_.isActive())
  {
    ROS_INFO("search_server_ is not active");
    return;
  }
  filter_boxes(msg);
  if (visualization)
  {

    display_places(boxes_on_the_right_side, "FilteredBoxes");
    area_of_interest_ = Box(Point(point_min_x, point_max_y), Point(point_min_x, point_min_y),
                            Point(point_max_x, point_max_y), Point(point_max_x, point_min_y));
    area_of_interest_.visualize(visualize_free_place, "Area of interest", 1, 1, 1);
  }
  // ROS_INFO("Size of  boxes_on_the_right %lu",boxes_on_the_right_side.size());

  switch (action_status.action_status)
  {
  case START_SEARCHING_PLACE:
    if (find_free_places())
    {
      publishFeedback(FOUND_PLACE_MEASURING);
      speed_current.data = speed_when_found_place;
      speed_publisher.publish(speed_current);
    }
    break;

  case FOUND_PLACE_MEASURING:
    if (find_free_places())
    {
      if (first_free_place.bottom_left.x <= max_distance_to_free_place_)
      {
        publishFeedback(FIND_PROPER_PLACE);
        speed_current.data = default_speed_in_parking_zone;
      }
      speed_publisher.publish(speed_current);
    } else
    {
      speed_current.data = default_speed_in_parking_zone;
      publishFeedback(START_SEARCHING_PLACE);
    }
    speed_publisher.publish(speed_current);
    break;

  case FIND_PROPER_PLACE:
    if (find_free_places())
    {
      std::cout << "Found proper place\nsending result";
      send_goal();
    } else
    {
      std::cout << "Place lost\n";
      speed_current.data = default_speed_in_parking_zone;
      publishFeedback(START_SEARCHING_PLACE);
      speed_publisher.publish(speed_current);
    }
    break;

  default:
    ROS_INFO("Err, wrong action_status");
    break;
  }
}

void Search_server::filter_boxes(const selfie_msgs::PolygonArray &msg)
{
  boxes_on_the_right_side.clear();
  for (int box_nr = msg.polygons.size() - 1; box_nr >= 0; box_nr--)
  {
    geometry_msgs::Polygon polygon = msg.polygons[box_nr];
    bool box_ok = false;
    for (int a = 0; a < 4; ++a)
    {
      Point p(polygon.points[a]);
      if (p.check_position(point_min_x, point_max_x, point_min_y, point_max_y))
      {
        box_ok = true;
        break;
      }
    }
    if (box_ok)
    {
      Box temp_box(polygon);
      if (abs(temp_box.left_vertical_line.a) < tangens_of_box_angle_ &&
          abs((temp_box.top_right.x - temp_box.bottom_right.x) - (temp_box.top_right.y - temp_box.bottom_right.y)) <
              tangens_of_box_angle_) // filters out boxes which are not parallel to car
        boxes_on_the_right_side.insert(boxes_on_the_right_side.begin(), temp_box);
    }
  }
}

bool Search_server::find_free_places()
{
  if (boxes_on_the_right_side.size() < 2)
  {
    // ROS_WARN("REJECTED");
    return false;
  }
  float min_space = min_spot_lenght;
  // ROS_INFO("min space: %f", min_space);
  vector<Box>::iterator iter = boxes_on_the_right_side.begin();
  vector<Box>::const_iterator end_iter = boxes_on_the_right_side.cend();
  for (; iter + 1 != end_iter; ++iter)
  {
    double dist = (*iter).top_left.get_distance((*(iter + 1)).bottom_left);
    // ROS_INFO("dist: %f", dist);
    if (dist > min_space)
    {

      Box tmp_box((*iter).top_left, (*iter).top_right, (*(iter + 1)).bottom_left, (*(iter + 1)).bottom_right);
      first_free_place = tmp_box;
      ROS_INFO("Found place \nTL: x=%lf y=%lf\nTR: x=%lf y=%lf\nBL x=%lf "
               "y=%lf\nBR x=%lf y=%lf\n",
               tmp_box.top_left.x, tmp_box.top_left.y, tmp_box.top_right.x, tmp_box.top_right.y, tmp_box.bottom_left.x,
               tmp_box.bottom_left.y, tmp_box.bottom_right.x, tmp_box.bottom_right.y);
      if (visualization)
      {
        display_place(tmp_box, "first_free_place");
      }
      return true;
    }
  }
  return false;
}

void Search_server::distanceCb(const std_msgs::Float32 &msg)
{
  current_distance_ = msg.data;
  if (!max_distance_calculated_)
  {
    max_distance_ = current_distance_ + length_of_parking_area_;
    max_distance_calculated_ = true;
  }
  if (max_distance_ < current_distance_)
  {
    ROS_INFO("Haven't found free place in designated distance, aborting");
    preemptCB();
  }
}

void Search_server::send_goal()
{

  geometry_msgs::Point32 p;
  result.parking_spot.points.clear();
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

  ROS_INFO("Place found and sent");
  search_server_.setSucceeded(result);
  endAction();
}

void Search_server::display_place(Box &place, const std::string &name, float r, float g, float b)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = name;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.lifetime = ros::Duration(1);

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
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

  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = name;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.lifetime = ros::Duration(1);

  marker.color.r = 0.0f;
  marker.color.g = 255.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;

  geometry_msgs::Point marker_point;
  marker_point.z = 0;
  for (int i = boxes.size() - 1; i >= 0; i--)
  {
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
void Search_server::publishFeedback(unsigned int newActionStatus)
{
  action_status.action_status = newActionStatus;
  search_server_.publishFeedback(action_status);
}

void Search_server::preemptCB()
{
  ROS_INFO("Action preempted");
  endAction();
  search_server_.setAborted();
}

void Search_server::endAction() // shutting donw unnecesary subscribers and publishers
{
  obstacles_sub.shutdown();
  distance_sub_.shutdown();
  changeSetpoint(old_setpoint_);
  max_distance_calculated_ = false;
}

void Search_server::changeSetpoint(float setpoint)
{
  conf_.doubles.clear();
  double_param_.name = "right_lane_setpoint";
  double_param_.value = setpoint;
  conf_.doubles.push_back(double_param_);

  srv_req_.config = conf_;

  ros::service::call("/lane_controller/set_parameters", srv_req_, srv_resp_);

  pnh_.setParam("/lane_controller/right_lane_setpoint", setpoint);
}

void Search_server::reconfigureCB(selfie_park::DetectParkingSpotConfig &config, uint32_t level)
{
  if (default_speed_in_parking_zone != (float)config.default_speed_in_parking_zone)
  {
    default_speed_in_parking_zone = config.default_speed_in_parking_zone;
    ROS_INFO("default_speed in parking_zone new value: %f", default_speed_in_parking_zone);
  }
  if (max_distance_to_free_place_ != (float)config.max_distance_to_free_place)
  {
    max_distance_to_free_place_ = config.max_distance_to_free_place;
    ROS_INFO("max_distance_to_free_place_ new value: %f", max_distance_to_free_place_);
  }
  if (speed_when_found_place != (float)config.speed_when_found_place)
  {
    speed_when_found_place = config.speed_when_found_place;
    ROS_INFO("speed_when_found_place new value: %f", speed_when_found_place);
  }
  if (new_setpoint_ != (float)config.new_setpoint)
  {
    new_setpoint_ = config.new_setpoint;
    ROS_INFO("new_setpoint new value: %f", new_setpoint_);
  }
}
