/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <selfie_perception/lane_detector.h>
#include <vector>
#include <algorithm>

#define TOPVIEW_ROWS 480
#define TOPVIEW_COLS 640

#define TOPVIEW_MIN_X 0.3
#define TOPVIEW_MAX_X 1.3
#define TOPVIEW_MIN_Y -1.2
#define TOPVIEW_MAX_Y 1.2

static int Acc_slider = 1;
static double Acc_value = 0.7;
static int Acc_filt = 1;
static int Acc_filt_slider = 40;

LaneDetector::LaneDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
  nh_(nh),
  pnh_(pnh),
  it_(nh),
  debug_mode_(false),
  init_imageCallback_(true),
  max_mid_line_gap_(0.8),
  max_mid_line_distance_(0.12),
  min_length_search_line_(0.10),
  max_delta_y_lane_(0.08),
  nominal_center_line_Y_(0.2),
  min_length_to_aprox_(0.56),
  poly_nDegree_(2),
  left_lane_width_(0.4),
  right_lane_width_(0.4),
  points_density_(15),
  left_line_index_(-1),
  right_line_index_(-1),
  center_line_index_(-1),
  short_left_line_(false),
  short_center_line_(false),
  short_right_line_(false),
  real_window_size_(0.1),
  threshold_c_(-40)

{
  lanes_pub_ = nh_.advertise<selfie_msgs::RoadMarkings>("road_markings", 100);
  intersection_pub_ = nh_.advertise<std_msgs::Float32>("intersection_distance", 100);
  starting_line_pub_ = nh_.advertise<std_msgs::Float32>("starting_line", 100);
}

LaneDetector::~LaneDetector()
{
  cv::destroyAllWindows();
}

bool LaneDetector::init()
{
  lanes_vector_.clear();
  aprox_lines_frame_coordinate_.clear();
  std::vector<cv::Point2f> empty;
  empty.clear();
  aprox_lines_frame_coordinate_.push_back(empty);
  aprox_lines_frame_coordinate_.push_back(empty);
  aprox_lines_frame_coordinate_.push_back(empty);

  kernel_v_ = cv::Mat(1, 3, CV_32F);
  kernel_v_.at<float>(0, 0) = -1;
  kernel_v_.at<float>(0, 1) = 0;
  kernel_v_.at<float>(0, 2) = 1;

  pnh_.getParam("config_file", config_file_);
  pnh_.getParam("real_window_size", real_window_size_);
  pnh_.getParam("threshold_c", threshold_c_);
  pnh_.getParam("debug_mode", debug_mode_);
  pnh_.getParam("max_mid_line_gap", max_mid_line_gap_);

  treshold_block_size_ = static_cast<int>(TOPVIEW_COLS / (TOPVIEW_MAX_Y - TOPVIEW_MIN_Y) * real_window_size_);
  if (treshold_block_size_ % 2 == 0)
    treshold_block_size_++;

  image_sub_ = it_.subscribe("/image_rect", 10, &LaneDetector::imageCallback, this);
  if (debug_mode_)
  {
    points_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("new_coordinates", 10);
    aprox_visualization_pub_ = nh_.advertise<visualization_msgs::Marker>("aprox", 10);
  }

  cv::FileStorage fs(config_file_, cv::FileStorage::READ);
  fs["world2cam"] >> world2cam_;
  fs.release();

  computeTopView();

  printInfoParams();
  return true;
}

void LaneDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    current_frame_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  homography(current_frame_, homography_frame_);
  // removeCar(homography_frame_);

  cv::adaptiveThreshold(homography_frame_, binary_frame_, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                        CV_THRESH_BINARY, treshold_block_size_, threshold_c_);

  if (!init_imageCallback_)
  {
    dynamicMask(binary_frame_, masked_frame_);
    if (debug_mode_)
      cv::bitwise_and(homography_frame_, dynamic_mask_, homography_frame_);

    // remove ROI inside left and right lane
    ROILaneRight(masked_frame_, masked_frame_);
    ROILaneLeft(masked_frame_, masked_frame_);
    detectStartAndIntersectionLine();
  }
  else
    masked_frame_ = binary_frame_.clone();

  cv::medianBlur(masked_frame_, masked_frame_, 5);
  cv::filter2D(masked_frame_, canny_frame_, -1, kernel_v_, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

  detectLines(canny_frame_, lanes_vector_);
  if (lanes_vector_.empty())
  {
    left_line_index_ = -1;
    center_line_index_ = -1;
    right_line_index_ = -1;
    return;
  }
  convertCoordinates();
  filterSmallLines();
  if (lanes_vector_converted_.empty())
  {
    left_line_index_ = -1;
    center_line_index_ = -1;
    right_line_index_ = -1;
    return;
  }

  mergeMiddleLane();

  if (debug_mode_)
  {
    debug_frame_.rows = homography_frame_.rows;
    debug_frame_.cols = homography_frame_.cols;
    lanesVectorVisualization(debug_frame_);
  }

  if (init_imageCallback_)
  {
    initRecognizeLines();
    int l = 0, c = 0, r = 0;
    if (left_line_index_ != -1)
      if (cv::arcLength(lanes_vector_converted_[left_line_index_], false) > min_length_to_aprox_)
        l = 1;

    if (center_line_index_ != -1)
      if (cv::arcLength(lanes_vector_converted_[center_line_index_], false) > min_length_to_aprox_)
        c = 1;

    if (right_line_index_ != -1)
      if (cv::arcLength(lanes_vector_converted_[right_line_index_], false) > min_length_to_aprox_)
        r = 1;

    if ((l + r + c) > 1)
    {
      linesApproximation(lanes_vector_converted_);
      init_imageCallback_ = false;
    }
  }
  else
  {
    recognizeLines();
    generatePoints();

    calcRoadWidth();
    addBottomPoint();
    linesApproximation(lanes_vector_converted_);
    publishMarkings();
  }

  if (debug_mode_)
  {
    visualization_frame_.rows = homography_frame_.rows;
    visualization_frame_.cols = homography_frame_.cols;

    convertApproxToFrameCoordinate();
    drawPoints(visualization_frame_);

    openCVVisualization();
    aproxVisualization();
    pointsRVIZVisualization();
  }
}

bool LaneDetector::resetVisionCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  init_imageCallback_ = true;
  ROS_INFO("RESET VISION");
  return true;
}

void LaneDetector::computeTopView()
{
  // Choose top-view image size
  topview_size_ = cv::Size(TOPVIEW_COLS, TOPVIEW_ROWS);

  // Choose corner points (in real-world coordinates)
  std::vector<cv::Point2f> coordinates;
  coordinates.emplace_back(TOPVIEW_MIN_X, TOPVIEW_MIN_Y);
  coordinates.emplace_back(TOPVIEW_MIN_X, TOPVIEW_MAX_Y);
  coordinates.emplace_back(TOPVIEW_MAX_X, TOPVIEW_MIN_Y);
  coordinates.emplace_back(TOPVIEW_MAX_X, TOPVIEW_MAX_Y);

  std::vector<cv::Point2f> pixels;
  pixels.emplace_back(topview_size_.width, topview_size_.height);
  pixels.emplace_back(0, topview_size_.height);
  pixels.emplace_back(topview_size_.width, 0);
  pixels.emplace_back(0, 0);

  topview2world_ = cv::findHomography(pixels, coordinates);
  world2topview_ = topview2world_.inv();

  topview2cam_ = world2cam_ * topview2world_;
}

void LaneDetector::detectLines(cv::Mat &input_frame, std::vector<std::vector<cv::Point>> &output_lanes)
{
  output_lanes.clear();
  cv::findContours(input_frame, output_lanes, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_L1, cv::Point(0, 0));

  for (std::vector<std::vector<cv::Point>>::iterator filt = output_lanes.begin(); filt != output_lanes.end();)
  {
    if (filt->size() < Acc_filt)
      filt = output_lanes.erase(filt);
    else
      ++filt;
  }
  for (int i = 0; i < output_lanes.size(); i++)
  {
    cv::approxPolyDP(cv::Mat(output_lanes[i]), output_lanes[i], Acc_value, false);
  }
}

void LaneDetector::drawPoints(cv::Mat &frame)
{
  frame = cv::Mat::zeros(frame.size(), CV_8UC3);
  if (!aprox_lines_frame_coordinate_[2].empty())
    for (int i = 0; i < aprox_lines_frame_coordinate_[2].size(); i++)
    {
      cv::circle(frame, aprox_lines_frame_coordinate_[2][i], 3, cv::Scalar(255, 0, 0), CV_FILLED, cv::LINE_AA);
    }
  if (!aprox_lines_frame_coordinate_[0].empty())
    for (int i = 0; i < aprox_lines_frame_coordinate_[0].size(); i++)
    {
      cv::circle(frame, aprox_lines_frame_coordinate_[0][i], 3, cv::Scalar(0, 0, 255), CV_FILLED, cv::LINE_AA);
    }
  if (!aprox_lines_frame_coordinate_[1].empty())
    for (int i = 0; i < aprox_lines_frame_coordinate_[1].size(); i++)
    {
      cv::circle(frame, aprox_lines_frame_coordinate_[1][i], 3, cv::Scalar(0, 255, 0), CV_FILLED, cv::LINE_AA);
    }
}

void LaneDetector::homography(cv::Mat input_frame, cv::Mat &homography_frame)
{
  cv::warpPerspective(input_frame, homography_frame, topview2cam_,
                      topview_size_, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
}

void LaneDetector::openCVVisualization()
{
  cv::namedWindow("Binarization", cv::WINDOW_NORMAL);
  cv::imshow("Binarization", binary_frame_);

  cv::namedWindow("masked", cv::WINDOW_NORMAL);
  cv::imshow("masked", masked_frame_);

  cv::namedWindow("Output", cv::WINDOW_NORMAL);
  cv::imshow("Output", visualization_frame_);

  cv::namedWindow("Homography", cv::WINDOW_NORMAL);
  cv::imshow("Homography", homography_frame_);

  cv::namedWindow("Debug", cv::WINDOW_NORMAL);
  cv::imshow("Debug", debug_frame_);

  cv::waitKey(1);
}

void LaneDetector::quickSortLinesY(int left, int right)
{
  int i = left;
  int j = right;
  int y = lanes_vector_[(left + right) / 2][0].y;
  do
  {
    while (lanes_vector_[i][0].y > y)
      i++;

    while (lanes_vector_[j][0].y < y)
      j--;

    if (i <= j)
    {
      lanes_vector_[i].swap(lanes_vector_[j]);

      i++;
      j--;
    }
  }
  while (i <= j);

  if (left < j)
    LaneDetector::quickSortLinesY(left, j);

  if (right > i)
    LaneDetector::quickSortLinesY(i, right);
}

void LaneDetector::quickSortPointsY(std::vector<cv::Point> &vector_in, int left, int right)
{
  int i = left;
  int j = right;
  int y = vector_in[(left + right) / 2].y;
  do
  {
    while (vector_in[i].y > y)
      i++;

    while (vector_in[j].y < y)
      j--;

    if (i <= j)
    {
      cv::Point temp = vector_in[i];
      vector_in[i] = vector_in[j];
      vector_in[j] = temp;

      i++;
      j--;
    }
  }
  while (i <= j);

  if (left < j)
    LaneDetector::quickSortPointsY(vector_in, left, j);

  if (right > i)
    LaneDetector::quickSortPointsY(vector_in, i, right);
}

void LaneDetector::mergeMiddleLane()
{
  for (int i = 0; i < lanes_vector_converted_.size(); i++)
  {
    float a, b;
    for (int j = i + 1; j < lanes_vector_converted_.size(); j++)
    {
      if (lanes_vector_converted_[i][lanes_vector_converted_[i].size() - 1].x - lanes_vector_converted_[i][0].x != 0)
        a = (lanes_vector_converted_[i][0].y - lanes_vector_converted_[i][lanes_vector_converted_[i].size() - 1].y)
            / (lanes_vector_converted_[i][0].x - lanes_vector_converted_[i][lanes_vector_converted_[i].size() - 1].x);
      else
        a = 999999;
      b = lanes_vector_converted_[i][0].y - a * lanes_vector_converted_[i][0].x;

      float distance = std::abs(a * lanes_vector_converted_[j][0].x - lanes_vector_converted_[j][0].y + b)
                       / sqrtf(a * a + 1);
      float distance2 = getDistance(lanes_vector_converted_[j][0],
                                    lanes_vector_converted_[i][lanes_vector_converted_[i].size() - 1]);

      if (distance < max_mid_line_distance_ && distance2 < max_mid_line_gap_)
      {
        lanes_vector_converted_[i].insert(lanes_vector_converted_[i].end(),
                                          lanes_vector_converted_[j].begin(), lanes_vector_converted_[j].end());
        lanes_vector_converted_.erase(lanes_vector_converted_.begin() + j);
        j--;
      }
    }
  }
}

float LaneDetector::getDistance(cv::Point2f p1, cv::Point2f p2)
{
  float dx = (p1.x - p2.x);
  float dy = (p1.y - p2.y);
  return sqrtf(dx * dx + dy * dy);
}

void LaneDetector::recognizeLines()
{
  float min_left = current_frame_.cols;
  int min_index = -1;
  std::vector<std::vector<cv::Point2f>> closest_container;
  std::vector<int> closest_index;
  for (int i = 0; i < lanes_vector_converted_.size(); i++)
  {
    float aprox_y = getAproxY(last_left_coeff_, lanes_vector_converted_[i][0].x);
    if (std::abs(aprox_y - lanes_vector_converted_[i][0].y) < max_delta_y_lane_)
    {
      closest_container.push_back(lanes_vector_converted_[i]);
      closest_index.push_back(i);
    }
  }
  if (closest_container.empty())
    left_line_index_ = -1;
  else
  {
    float aprox_y = getAproxY(last_left_coeff_, closest_container[0][0].x);
    float min_sum_distance = std::abs(aprox_y - closest_container[0][0].y);
    aprox_y = getAproxY(last_left_coeff_, closest_container[0][closest_container[0].size() / 2].x);
    min_sum_distance += std::abs(aprox_y - closest_container[0][closest_container[0].size() / 2].y);
    left_line_index_ = closest_index[0];

    for (int i = 1; i < closest_container.size(); i++)
    {
      aprox_y = getAproxY(last_left_coeff_, closest_container[i][0].x);
      float sum_distance = std::abs(aprox_y - closest_container[i][0].y);
      aprox_y = getAproxY(last_left_coeff_, closest_container[i][closest_container[i].size() / 2].x);
      sum_distance += std::abs(aprox_y - closest_container[i][closest_container[i].size() / 2].y);
      if (sum_distance < min_sum_distance)
      {
        left_line_index_ = closest_index[i];
        min_sum_distance = sum_distance;
      }
    }
  }
  closest_container.clear();
  closest_index.clear();

  for (int i = 0; i < lanes_vector_converted_.size(); i++)
  {
    if (i == left_line_index_)
      continue;
    float aprox_y = getAproxY(last_right_coeff_, lanes_vector_converted_[i][0].x);
    if (std::abs(aprox_y - lanes_vector_converted_[i][0].y) < max_delta_y_lane_)
    {
      closest_container.push_back(lanes_vector_converted_[i]);
      closest_index.push_back(i);
    }
  }
  if (closest_container.empty())
    right_line_index_ = -1;
  else
  {
    float aprox_y = getAproxY(last_right_coeff_, closest_container[0][0].x);
    float min_sum_distance = std::abs(aprox_y - closest_container[0][0].y);
    aprox_y = getAproxY(last_right_coeff_, closest_container[0][closest_container[0].size() / 2].x);
    min_sum_distance += std::abs(aprox_y - closest_container[0][closest_container[0].size() / 2].y);
    right_line_index_ = closest_index[0];

    for (int i = 1; i < closest_container.size(); i++)
    {
      aprox_y = getAproxY(last_right_coeff_, closest_container[i][0].x);
      float sum_distance = std::abs(aprox_y - closest_container[i][0].y);
      aprox_y = getAproxY(last_right_coeff_, closest_container[i][closest_container[i].size() / 2].x);
      sum_distance += std::abs(aprox_y - closest_container[i][closest_container[i].size() / 2].y);
      if (sum_distance < min_sum_distance)
      {
        right_line_index_ = closest_index[i];
        min_sum_distance = sum_distance;
      }
    }
  }
  closest_container.clear();
  closest_index.clear();

  for (int i = 0; i < lanes_vector_converted_.size(); i++)
  {
    if (i == left_line_index_ || i == right_line_index_)
      continue;
    float aprox_y = getAproxY(last_middle_coeff_, lanes_vector_converted_[i][0].x);
    if (std::abs(aprox_y - lanes_vector_converted_[i][0].y) < max_delta_y_lane_)
    {
      closest_container.push_back(lanes_vector_converted_[i]);
      closest_index.push_back(i);
    }
  }
  if (closest_container.empty())
    center_line_index_ = -1;
  else
  {
    float aprox_y = getAproxY(last_middle_coeff_, closest_container[0][0].x);
    float min_sum_distance = std::abs(aprox_y - closest_container[0][0].y);
    aprox_y = getAproxY(last_middle_coeff_, closest_container[0][closest_container[0].size() / 2].x);
    min_sum_distance += std::abs(aprox_y - closest_container[0][closest_container[0].size() / 2].y);
    center_line_index_ = closest_index[0];

    for (int i = 1; i < closest_container.size(); i++)
    {
      aprox_y = getAproxY(last_middle_coeff_, closest_container[i][0].x);
      float sum_distance = std::abs(aprox_y - closest_container[i][0].y);
      aprox_y = getAproxY(last_middle_coeff_, closest_container[i][closest_container[i].size() / 2].x);
      sum_distance += std::abs(aprox_y - closest_container[i][closest_container[i].size() / 2].y);
      if (sum_distance < min_sum_distance)
      {
        center_line_index_ = closest_index[i];
        min_sum_distance = sum_distance;
      }
    }
  }
}

void LaneDetector::publishMarkings()
{
  selfie_msgs::RoadMarkings road_markings;
  road_markings.header.stamp = ros::Time::now();
  road_markings.header.frame_id = "road_markings";
  for (int i = 0; i < left_coeff_.size(); i++)
  {
    road_markings.left_line.push_back(left_coeff_[i]);
    road_markings.right_line.push_back(right_coeff_[i]);
    road_markings.center_line.push_back(middle_coeff_[i]);
  }
  lanes_pub_.publish(road_markings);
}

void LaneDetector::printInfoParams()
{
  ROS_INFO("real_window_size: %.3f", real_window_size_);
  ROS_INFO("treshold_block_size: %d", treshold_block_size_);
  ROS_INFO("threshold_c: %d", threshold_c_);
  ROS_INFO("max_mid_line_gap: %.3f", max_mid_line_gap_);

  ROS_INFO("debug_mode: %d\n", debug_mode_);
}

void LaneDetector::dynamicMask(cv::Mat &input_frame, cv::Mat &output_frame)
{
  dynamic_mask_ = cv::Mat::zeros(cv::Size(input_frame.cols, input_frame.rows), CV_8UC1);
  float offset_right = -0.05;
  float offset_left = 0.03;
  output_frame = input_frame.clone();
  if (right_line_index_ == -1)
    offset_right = -0.14;
  else if (lanes_vector_converted_[right_line_index_][lanes_vector_converted_[right_line_index_].size() - 1].x
            < ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 4))
    offset_right = -0.14;
  if (left_line_index_ == -1)
    offset_left = 0.12;
  else if (lanes_vector_converted_[left_line_index_][lanes_vector_converted_[left_line_index_].size() - 1].x
            < ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 4))
    offset_left = 0.12;

  std::vector<cv::Point2f> left_line = createOffsetLine(left_coeff_, offset_left);
  std::vector<cv::Point2f> right_line = createOffsetLine(right_coeff_, offset_right);
  cv::transform(left_line, left_line, world2topview_.rowRange(0, 2));
  cv::transform(right_line, right_line, world2topview_.rowRange(0, 2));

  int l, r;
  const int KLength = left_line.size() + right_line.size();
  cv::Point points[KLength];
  for (l = 0; l < left_line.size(); l++)
  {
    points[l] = cv::Point(left_line[l].x, left_line[l].y);
  }
  for (r = right_line.size() - 1; r >= 0; r--)
  {
    points[l] = cv::Point(right_line[r].x, right_line[r].y);
    l++;
  }
  cv::fillConvexPoly(dynamic_mask_, points, KLength, cv::Scalar(255, 255, 255));
  cv::bitwise_and(input_frame, dynamic_mask_, output_frame);
}

void LaneDetector::ROILaneRight(cv::Mat &input_frame, cv::Mat &output_frame)
{
  right_lane_ROI_ = cv::Mat::zeros(cv::Size(input_frame.cols, input_frame.rows), CV_8UC1);
  output_frame = input_frame.clone();
  float offset_center = -0.07;
  float offset_right = 0.05;
  if (right_line_index_ == -1 || short_right_line_)
    offset_right = 0.11;
  if (center_line_index_ == -1 || short_center_line_)
    offset_center = -0.13;

  std::vector<cv::Point2f> center_line = createOffsetLine(middle_coeff_, offset_center);
  std::vector<cv::Point2f> right_line = createOffsetLine(right_coeff_, offset_right);
  cv::transform(center_line, center_line, world2topview_.rowRange(0, 2));
  cv::transform(right_line, right_line, world2topview_.rowRange(0, 2));

  int c, r;
  const int KLength = center_line.size() + right_line.size();
  cv::Point points[KLength];
  for (c = 0; c < center_line.size(); c++)
  {
    points[c] = cv::Point(center_line[c].x, center_line[c].y);
  }
  for (r = right_line.size() - 1; r >= 0; r--)
  {
    points[c] = cv::Point(right_line[r].x, right_line[r].y);
    c++;
  }

  cv::fillConvexPoly(right_lane_ROI_, points, KLength, cv::Scalar(255, 255, 255));

  right_lane_frame_ = input_frame.clone();
  cv::bitwise_and(input_frame, right_lane_ROI_, right_lane_frame_);

  cv::bitwise_not(right_lane_ROI_, right_lane_ROI_);
  cv::bitwise_and(input_frame, right_lane_ROI_, output_frame);
}

void LaneDetector::ROILaneLeft(cv::Mat &input_frame, cv::Mat &output_frame)
{
  left_lane_ROI_ = cv::Mat::zeros(cv::Size(input_frame.cols, input_frame.rows), CV_8UC1);
  output_frame = input_frame.clone();
  float offset_center = 0.05;
  float offset_left = -0.07;
  if (left_line_index_ == -1 || short_left_line_)
    offset_left = -0.13;
  if (center_line_index_ == -1 || short_center_line_)
    offset_center = 0.11;

  std::vector<cv::Point2f> center_line = createOffsetLine(middle_coeff_, offset_center);
  std::vector<cv::Point2f> left_line = createOffsetLine(left_coeff_, offset_left);
  cv::transform(center_line, center_line, world2topview_.rowRange(0, 2));
  cv::transform(left_line, left_line, world2topview_.rowRange(0, 2));

  int c, l;
  const int KLength = center_line.size() + left_line.size();
  cv::Point points[KLength];
  for (c = 0; c < center_line.size(); c++)
  {
    points[c] = cv::Point(center_line[c].x, center_line[c].y);
  }
  for (l = left_line.size() - 1; l >= 0; l--)
  {
    points[c] = cv::Point(left_line[l].x, left_line[l].y);
    c++;
  }

  cv::fillConvexPoly(left_lane_ROI_, points, KLength, cv::Scalar(255, 255, 255));

  left_lane_frame_ = input_frame.clone();
  cv::bitwise_and(input_frame, left_lane_ROI_, left_lane_frame_);

  cv::bitwise_not(left_lane_ROI_, left_lane_ROI_);
  cv::bitwise_and(input_frame, left_lane_ROI_, output_frame);
}

void LaneDetector::filterSmallLines()
{
  for (int i = 0; i < lanes_vector_converted_.size(); i++)
  {
    float sum = arcLength(lanes_vector_converted_[i], false);
    if (sum < min_length_search_line_)
    {
      lanes_vector_converted_.erase(lanes_vector_converted_.begin() + i);
      i--;
    }
  }
}

void LaneDetector::convertCoordinates()
{
  for (int i = 0; i < lanes_vector_.size(); i++)
  {
    quickSortPointsY(lanes_vector_[i], 0, lanes_vector_[i].size() - 1);
  }
  quickSortLinesY(0, lanes_vector_.size() - 1);

  lanes_vector_converted_.clear();
  for (int i = 0; i < lanes_vector_.size(); i++)
  {
    std::vector<cv::Point2f> line;
    cv::Mat(lanes_vector_[i]).copyTo(line);

    cv::transform(line, line, topview2world_.rowRange(0, 2));

    lanes_vector_converted_.push_back(line);
  }
}

float LaneDetector::getAproxY(std::vector<float> coeff, float x)
{
  size_t nDegree = coeff.size();

  float nY = 0;
  float nXdouble = 1;
  float nX = x;

  for (size_t j = 0; j < nDegree; j++)
  {
    // multiply current x by a coefficient
    nY += coeff[j] * nXdouble;
    // power up the X
    nXdouble *= nX;
  }
  return nY;
}

void LaneDetector::convertApproxToFrameCoordinate()
{
  aprox_lines_frame_coordinate_[0].clear();
  aprox_lines_frame_coordinate_[1].clear();
  aprox_lines_frame_coordinate_[2].clear();

  cv::Point2f p;
  float increment = 0.1;
  for (float x = TOPVIEW_MIN_X; x < TOPVIEW_MAX_X; x += increment)
  {
    p.x = x;

    p.y = getAproxY(left_coeff_, x);
    aprox_lines_frame_coordinate_[0].push_back(p);

    p.y = getAproxY(middle_coeff_, x);
    aprox_lines_frame_coordinate_[1].push_back(p);

    p.y = getAproxY(right_coeff_, x);
    aprox_lines_frame_coordinate_[2].push_back(p);
  }
  cv::transform(aprox_lines_frame_coordinate_[0], aprox_lines_frame_coordinate_[0], world2topview_.rowRange(0, 2));
  cv::transform(aprox_lines_frame_coordinate_[1], aprox_lines_frame_coordinate_[1], world2topview_.rowRange(0, 2));
  cv::transform(aprox_lines_frame_coordinate_[2], aprox_lines_frame_coordinate_[2], world2topview_.rowRange(0, 2));
}

void LaneDetector::initRecognizeLines()
{
  // check lines growth
  for (int i = 0; i < lanes_vector_converted_.size(); i++)
  {
    for (int j = 1; j < lanes_vector_converted_[i].size(); j++)
    {
      if (lanes_vector_converted_[i][j - 1].x - lanes_vector_converted_[i][j].x > 0)
      {
        lanes_vector_converted_[i].erase(lanes_vector_converted_[i].begin() + j, lanes_vector_converted_[i].end());
        break;
      }
    }
  }

  float min = homography_frame_.cols;
  int min_index = -1;
  for (int i = 0; i < lanes_vector_converted_.size(); i++)
  {
    if (std::abs(nominal_center_line_Y_ - lanes_vector_converted_[i][0].y) < min && lanes_vector_converted_[i][0].x
        < 1 && cv::arcLength(lanes_vector_converted_[i], false) > 0.15)
    {
      min = std::abs(nominal_center_line_Y_ - lanes_vector_converted_[i][0].y);
      min_index = i;
    }
  }
  center_line_index_ = min_index;

  if (center_line_index_ == -1)
  {
    right_line_index_ = -1;
    left_line_index_ = -1;
    return;
  }

  min = homography_frame_.cols;
  min_index = -1;
  for (int i = 0; i < lanes_vector_converted_.size(); i++)
  {
    if (lanes_vector_converted_[i][0].y < lanes_vector_converted_[center_line_index_][0].y &&
        lanes_vector_converted_[i][0].x < 1 && cv::arcLength(lanes_vector_converted_[i], false) > 0.2)
    {
      float distance_last = getDistance(lanes_vector_converted_[center_line_index_][0], lanes_vector_converted_[i][0]);
      float distance;
      for (int j = 1; j < lanes_vector_converted_[i].size(); j++)
      {
        distance = getDistance(lanes_vector_converted_[center_line_index_][0], lanes_vector_converted_[i][j]);
        if (distance_last - distance < 0)
          break;
        distance_last = distance;
      }
      if (distance_last < min && distance_last > 0.2 && distance_last < 0.5)
      {
        min = distance_last;
        min_index = i;
      }
    }
  }
  right_line_index_ = min_index;

  min = homography_frame_.cols;
  min_index = -1;
  for (int i = 0; i < lanes_vector_converted_.size(); i++)
  {
    if (lanes_vector_converted_[i][0].y > lanes_vector_converted_[center_line_index_][0].y &&
        lanes_vector_converted_[i][0].x < 1 && cv::arcLength(lanes_vector_converted_[i], false) > 0.2)
    {
      float distance_last = getDistance(lanes_vector_converted_[center_line_index_][0], lanes_vector_converted_[i][0]);
      float distance;
      for (int j = 1; j < lanes_vector_converted_[i].size(); j++)
      {
        distance = getDistance(lanes_vector_converted_[center_line_index_][0], lanes_vector_converted_[i][j]);
        if (distance_last - distance < 0)
          break;
        distance_last = distance;
      }
      if (distance_last < min && distance_last > 0.2 && distance_last < 0.5)
      {
        min = distance_last;
        min_index = i;
      }
    }
  }
  left_line_index_ = min_index;
}

void LaneDetector::pointsRVIZVisualization()
{
  geometry_msgs::Point32 point;
  point.z = 0;
  points_cloud_.points.clear();
  points_cloud_.header.frame_id = "road_markings";

  for (int i = 0; i < lanes_vector_converted_.size(); i++)
  {
    for (int j = 0; j < lanes_vector_converted_[i].size(); j++)
    {
      point.x = lanes_vector_converted_[i][j].x;
      point.y = lanes_vector_converted_[i][j].y;
      points_cloud_.points.push_back(point);
    }
  }

  points_cloud_pub_.publish(points_cloud_);
}

void LaneDetector::aproxVisualization()
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "road_markings";
  marker.header.stamp = ros::Time::now();
  marker.ns = "line";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.lifetime = ros::Duration();

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.scale.x = 0.003;

  geometry_msgs::Point marker_point;
  marker_point.z = 0;

  float increment = 0.1;
  for (float x = TOPVIEW_MIN_X; x < TOPVIEW_MAX_X; x += increment)
  {
    marker_point.x = x;
    marker_point.y = getAproxY(left_coeff_, x);
    marker.points.push_back(marker_point);
  }
  for (float x = TOPVIEW_MIN_X; x < TOPVIEW_MAX_X; x += increment)
  {
    marker_point.x = x;
    marker_point.y = getAproxY(middle_coeff_, x);
    marker.points.push_back(marker_point);
  }
  for (float x = TOPVIEW_MIN_X; x < TOPVIEW_MAX_X; x += increment)
  {
    marker_point.x = x;
    marker_point.y = getAproxY(right_coeff_, x);
    marker.points.push_back(marker_point);
  }
  aprox_visualization_pub_.publish(marker);
}

void LaneDetector::linesApproximation(std::vector<std::vector<cv::Point2f>> lanes_vector)
{
  unsigned char l_state, c_state, r_state;
  left_coeff_.clear();
  middle_coeff_.clear();
  right_coeff_.clear();

  // vectors length count
  short_left_line_ = false;
  short_right_line_ = false;
  short_center_line_ = false;
  double left_length = 0, right_length = 0, middle_length = 0;

  if (left_line_index_ != -1)
    left_length = cv::arcLength(lanes_vector[left_line_index_], false);
  else
    left_length = 0;
  if (center_line_index_ != -1)
    middle_length = cv::arcLength(lanes_vector[center_line_index_], false);
  else
    middle_length = 0;
  if (right_line_index_ != -1)
    right_length = cv::arcLength(lanes_vector[right_line_index_], false);
  else
    right_length = 0;

  int suitable_lines = 3;

  if (left_length < min_length_to_aprox_)
  {
    short_left_line_ = true;
    suitable_lines--;
  }

  if (right_length < min_length_to_aprox_)
  {
    short_right_line_ = true;
    suitable_lines--;
  }

  if (middle_length < min_length_to_aprox_)
  {
    short_center_line_ = true;
    suitable_lines--;
  }

  switch (suitable_lines)
  {
  case 3:
    // "l+  c+  r+"
    l_state = '+';
    c_state = '+';
    r_state = '+';

    if (!polyfit(poly_nDegree_, lanes_vector[left_line_index_], left_coeff_))
      left_coeff_ = last_left_coeff_;
    if (!polyfit(poly_nDegree_, lanes_vector[center_line_index_], middle_coeff_))
      middle_coeff_ = last_middle_coeff_;
    if (!polyfit(poly_nDegree_, lanes_vector[right_line_index_], right_coeff_))
      right_coeff_ = last_right_coeff_;

    break;

  case 2:
    if (short_left_line_)
    {
      if (!polyfit(poly_nDegree_, lanes_vector[center_line_index_], middle_coeff_))
        middle_coeff_ = last_middle_coeff_;
      if (!polyfit(poly_nDegree_, lanes_vector[right_line_index_], right_coeff_))
        right_coeff_ = last_right_coeff_;

      if (left_line_index_ == -1)
      {
        // "l-  c+  r+"
        l_state = '-';
        c_state = '+';
        r_state = '+';
        if (!polyfit(poly_nDegree_, createOffsetLine(middle_coeff_, left_lane_width_), left_coeff_))
          left_coeff_ = last_left_coeff_;
      }
      else
      {
        // l/  c+  r+"
        l_state = '/';
        c_state = '+';
        r_state = '+';
        left_coeff_ = adjust(middle_coeff_, lanes_vector[left_line_index_], true);
      }
    }
    else if (short_right_line_)
    {
      if (!polyfit(poly_nDegree_, lanes_vector[center_line_index_], middle_coeff_))
        middle_coeff_ = last_middle_coeff_;
      if (!polyfit(poly_nDegree_, lanes_vector[left_line_index_], left_coeff_))
        left_coeff_ = last_left_coeff_;

      if (right_line_index_ == -1)
      {
        // "l+  c+  r-"
        l_state = '+';
        c_state = '+';
        r_state = '-';
        if (!polyfit(poly_nDegree_, createOffsetLine(middle_coeff_, -1 * right_lane_width_), right_coeff_))
          right_coeff_ = last_right_coeff_;
      }
      else
      {
        // "l+  c+  r/"
        l_state = '+';
        c_state = '+';
        r_state = '/';
        right_coeff_ = adjust(middle_coeff_, lanes_vector[right_line_index_], false);
      }
    }
    else if (short_center_line_)
    {
      if (!polyfit(poly_nDegree_, lanes_vector[left_line_index_], left_coeff_))
        left_coeff_ = last_left_coeff_;
      if (!polyfit(poly_nDegree_, lanes_vector[right_line_index_], right_coeff_))
        right_coeff_ = last_right_coeff_;

      if (center_line_index_ == -1)
      {
        // "l+  c-  r+"
        l_state = '+';
        c_state = '-';
        r_state = '+';
        if (!polyfit(poly_nDegree_, createOffsetLine(right_coeff_, right_lane_width_), middle_coeff_))
          middle_coeff_ = last_middle_coeff_;
      }
      else
      {
        // "l+  c/  r+"
        l_state = '+';
        c_state = '/';
        r_state = '+';
        middle_coeff_ = adjust(right_coeff_, lanes_vector[center_line_index_], true);
      }
    }
    break;

  case 1:
    if (!short_right_line_)
    {
      if (!polyfit(poly_nDegree_, lanes_vector[right_line_index_], right_coeff_))
        right_coeff_ = last_right_coeff_;
      if (center_line_index_ != -1 || left_line_index_ != -1)
      {
        if (center_line_index_ != -1 && left_line_index_ != -1)
        {
          // "l/  c/  r+"
          l_state = '/';
          c_state = '/';
          r_state = '+';
          middle_coeff_ = adjust(right_coeff_, lanes_vector[center_line_index_], true);

          left_coeff_ = adjust(right_coeff_, lanes_vector[left_line_index_], true);
        }
        else if (center_line_index_ != -1)
        {
          // "l-  c/  r+"
          l_state = '-';
          c_state = '/';
          r_state = '+';
          middle_coeff_ = adjust(right_coeff_, lanes_vector[center_line_index_], true);

          if (!polyfit(poly_nDegree_, createOffsetLine(middle_coeff_, left_lane_width_), left_coeff_))
            left_coeff_ = last_left_coeff_;
        }
        else
        {
          // "l/  c-  r+"
          l_state = '/';
          c_state = '-';
          r_state = '+';
          left_coeff_ = adjust(right_coeff_, lanes_vector[left_line_index_], true);

          if (!polyfit(poly_nDegree_, createOffsetLine(right_coeff_, right_lane_width_), middle_coeff_))
            middle_coeff_ = last_middle_coeff_;
        }
      }
      else
      {
        // "l-  c-  r+"
        l_state = '-';
        c_state = '-';
        r_state = '+';
        if (!polyfit(poly_nDegree_, createOffsetLine(right_coeff_, right_lane_width_), middle_coeff_))
          middle_coeff_ = last_middle_coeff_;
        if (!polyfit(poly_nDegree_, createOffsetLine(middle_coeff_, left_lane_width_), left_coeff_))
          left_coeff_ = last_left_coeff_;
      }
    }

    else if (!short_left_line_)
    {
      if (!polyfit(poly_nDegree_, lanes_vector[left_line_index_], left_coeff_))
        left_coeff_ = last_left_coeff_;
      if (center_line_index_ != -1 || right_line_index_ != -1)
      {
        if (center_line_index_ != -1 && right_line_index_ != -1)
        {
          // "l+  c/  r/"
          l_state = '+';
          c_state = '/';
          r_state = '/';
          middle_coeff_ = adjust(left_coeff_, lanes_vector[center_line_index_], false);

          right_coeff_ = adjust(left_coeff_, lanes_vector[right_line_index_], false);
        }
        else if (center_line_index_ != -1)
        {
          // "l+  c/  r-"
          l_state = '+';
          c_state = '/';
          r_state = '-';
          middle_coeff_ = adjust(left_coeff_, lanes_vector[center_line_index_], false);

          if (!polyfit(poly_nDegree_, createOffsetLine(middle_coeff_, -1 * right_lane_width_), right_coeff_))
            right_coeff_ = last_right_coeff_;
        }
        else
        {
          // "l+  c-  r/"
          l_state = '+';
          c_state = '-';
          r_state = '/';
          right_coeff_ = adjust(left_coeff_, lanes_vector[right_line_index_], false);

          if (!polyfit(poly_nDegree_, createOffsetLine(left_coeff_, -1 * left_lane_width_), middle_coeff_))
            middle_coeff_ = last_middle_coeff_;
        }
      }
      else
      {
        // "l+  c-  r-"
        l_state = '+';
        c_state = '-';
        r_state = '-';
        if (!polyfit(poly_nDegree_, createOffsetLine(left_coeff_, -1 * left_lane_width_), middle_coeff_))
          middle_coeff_ = last_middle_coeff_;
        if (!polyfit(poly_nDegree_, createOffsetLine(middle_coeff_, -1 * right_lane_width_), right_coeff_))
          right_coeff_ = last_right_coeff_;
      }
    }

    else if (!short_center_line_)
    {
      if (!polyfit(poly_nDegree_, lanes_vector[center_line_index_], middle_coeff_))
        middle_coeff_ = last_middle_coeff_;
      if (left_line_index_ != -1 || right_line_index_ != -1)
      {
        if (left_line_index_ != -1 && right_line_index_ != -1)
        {
          // "l/  c+  r/"
          l_state = '/';
          c_state = '+';
          r_state = '/';
          left_coeff_ = adjust(middle_coeff_, lanes_vector[left_line_index_], true);

          right_coeff_ = adjust(middle_coeff_, lanes_vector[right_line_index_], false);
        }
        else if (left_line_index_ != -1)
        {
          // "l/  c+  r-"
          l_state = '/';
          c_state = '+';
          r_state = '-';
          left_coeff_ = adjust(middle_coeff_, lanes_vector[left_line_index_], true);

          if (!polyfit(poly_nDegree_, createOffsetLine(middle_coeff_, -1 * right_lane_width_), right_coeff_))
            right_coeff_ = last_right_coeff_;
        }
        else
        {
          // "l-  c+  r/"
          l_state = '-';
          c_state = '+';
          r_state = '/';
          right_coeff_ = adjust(middle_coeff_, lanes_vector[right_line_index_], false);

          if (!polyfit(poly_nDegree_, createOffsetLine(middle_coeff_, left_lane_width_), left_coeff_))
            left_coeff_ = last_left_coeff_;
        }
      }
      else
      {
        // "l-  c+  r-"
        l_state = '-';
        c_state = '+';
        r_state = '-';
        if (!polyfit(poly_nDegree_, createOffsetLine(middle_coeff_, left_lane_width_), left_coeff_))
          left_coeff_ = last_left_coeff_;
        if (!polyfit(poly_nDegree_, createOffsetLine(middle_coeff_, -1 * right_lane_width_), right_coeff_))
          right_coeff_ = last_right_coeff_;
      }
    }
    break;

  case 0:
    // "l-  c-  r-"
    l_state = '-';
    c_state = '-';
    r_state = '-';
    if (!last_left_coeff_.empty())
      left_coeff_ = last_left_coeff_;
    if (!last_right_coeff_.empty())
      right_coeff_ = last_right_coeff_;
    if (!last_middle_coeff_.empty())
      middle_coeff_ = last_middle_coeff_;
    break;
  }

  last_left_coeff_.clear();
  last_middle_coeff_.clear();
  last_right_coeff_.clear();

  last_left_coeff_ = left_coeff_;
  last_middle_coeff_ = middle_coeff_;
  last_right_coeff_ = right_coeff_;

  if (debug_mode_)
  {
    std::cout << "--------aprox state---------" << std::endl;
    std::cout << "left_index   " << left_line_index_ << std::endl;
    std::cout << "middle_index " << center_line_index_ << std::endl;
    std::cout << "right_index  " << right_line_index_ << std::endl
              << std::endl;

    std::cout << "left_length   " << left_length << std::endl;
    std::cout << "middle_length " << middle_length << std::endl;
    std::cout << "right_length  " << right_length << std::endl
              << std::endl;

    std::cout << "l" << l_state << "  c" << c_state << "  r" << r_state << std::endl;
  }
}

void LaneDetector::lanesVectorVisualization(cv::Mat &visualization_frame)
{
  lanes_vector_.clear();
  for (int i = 0; i < lanes_vector_converted_.size(); i++)
  {
    std::vector<cv::Point2f> line2f;
    std::vector<cv::Point> line;
    cv::transform(lanes_vector_converted_[i], line2f, world2topview_.rowRange(0, 2));
    cv::Mat(line2f).copyTo(line);
    lanes_vector_.push_back(line);
  }

  visualization_frame = cv::Mat::zeros(visualization_frame.size(), CV_8UC3);
  if (lanes_vector_.empty())
    return;
  for (int i = 0; i < lanes_vector_.size(); i++)
  {
    for (int j = 0; j < lanes_vector_[i].size(); j++)
    {
      cv::circle(visualization_frame, lanes_vector_[i][j], 2, cv::Scalar(255, 0, 0), CV_FILLED, cv::LINE_AA);
    }
  }

  int red_value = 0;
  for (int i = 0; i < lanes_vector_.size(); i++)
  {
    if (i % 2 == 0)
      red_value = 0;
    else
      red_value = 255;
    for (int j = 1; j < lanes_vector_[i].size(); j++)
    {
      cv::line(visualization_frame, lanes_vector_[i][j], lanes_vector_[i][j - 1], cv::Scalar(red_value, 0, 255), 1);
    }
  }
}

void LaneDetector::removeCar(cv::Mat &frame)
{
  cv::Mat car_mask = cv::Mat::zeros(cv::Size(frame.cols, frame.rows), CV_8UC1);
  cv::Point points[4];
  points[0] = cv::Point(265, 480);
  points[1] = cv::Point(370, 480);
  points[2] = cv::Point(370, 280);
  points[3] = cv::Point(265, 280);

  cv::fillConvexPoly(car_mask, points, 4, cv::Scalar(255, 255, 255));
  cv::bitwise_not(car_mask, car_mask);
  cv::bitwise_and(frame, car_mask, frame);
}

void LaneDetector::addBottomPoint()
{
  cv::Point2f temp;
  if (left_line_index_ > -1)
  {
    if (lanes_vector_converted_[left_line_index_][0].x > ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 4))
    {
      temp.x = TOPVIEW_MIN_X;
      temp.y = getAproxY(last_left_coeff_, TOPVIEW_MIN_X);
      lanes_vector_converted_[left_line_index_].insert(lanes_vector_converted_[left_line_index_].begin(), temp);
    }
  }

  if (right_line_index_ > -1)
  {
    if (lanes_vector_converted_[right_line_index_][0].x > ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 4))
    {
      temp.x = TOPVIEW_MIN_X;
      temp.y = getAproxY(last_right_coeff_, TOPVIEW_MIN_X);
      lanes_vector_converted_[right_line_index_].insert(lanes_vector_converted_[right_line_index_].begin(), temp);
    }
  }

  if (center_line_index_ > -1)
  {
    if (lanes_vector_converted_[center_line_index_][0].x > ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 4))
    {
      temp.x = TOPVIEW_MIN_X;
      temp.y = getAproxY(last_middle_coeff_, TOPVIEW_MIN_X);
      lanes_vector_converted_[center_line_index_].insert(lanes_vector_converted_[center_line_index_].begin(), temp);
    }
  }
}

bool LaneDetector::polyfit(int nDegree, std::vector<cv::Point2f> line, std::vector<float> &coeff)
{
  if (line.size() < nDegree)
    return false;

  // more intuative this way
  nDegree++;

  size_t nCount = line.size();
  boost::numeric::ublas::matrix<float> oXMatrix(nCount, nDegree);
  boost::numeric::ublas::matrix<float> oYMatrix(nCount, 1);

  // copy y matrix
  for (size_t i = 0; i < nCount; i++)
  {
    oYMatrix(i, 0) = line[i].y;
  }

  // create the X matrix
  for (size_t nRow = 0; nRow < nCount; nRow++)
  {
    float nVal = 1.0f;
    for (int nCol = 0; nCol < nDegree; nCol++)
    {
      oXMatrix(nRow, nCol) = nVal;
      nVal *= line[nRow].x;
    }
  }
  // transpose X matrix
  boost::numeric::ublas::matrix<float> oXtMatrix(trans(oXMatrix));
  // multiply transposed X matrix with X matrix
  boost::numeric::ublas::matrix<float> oXtXMatrix(prec_prod(oXtMatrix, oXMatrix));
  // multiply transposed X matrix with Y matrix
  boost::numeric::ublas::matrix<float> oXtYMatrix(prec_prod(oXtMatrix, oYMatrix));

  // lu decomposition
  boost::numeric::ublas::permutation_matrix<int> pert(oXtXMatrix.size1());
  const std::size_t singular = lu_factorize(oXtXMatrix, pert);

  // must be singular
  if (singular != 0)
    return false;

  // backsubstitution
  lu_substitute(oXtXMatrix, pert, oXtYMatrix);

  // copy the result to coeff

  std::vector<float> vec(oXtYMatrix.data().begin(), oXtYMatrix.data().end());
  coeff = vec;
  return true;
}

std::vector<float> LaneDetector::adjust(std::vector<float> good_poly_coeff,
                                        std::vector<cv::Point2f> line, bool left_offset)
{
  std::vector<float> coeff;
  float a, b, width;
  if (line[1].y - line[line.size() - 1].y != 0)
  {
    a = -1 * (line[1].x - line[line.size() - 1].x) / (line[1].y - line[line.size() - 1].y);
    b = line[line.size() / 2].y - a * line[line.size() / 2].x;

    float delta = ((good_poly_coeff[1] - a) * (good_poly_coeff[1] - a))
                  - 4 * (good_poly_coeff[2] * (good_poly_coeff[0] - b));
    float x1 = (-1 * (good_poly_coeff[1] - a) - sqrtf(delta)) / (2 * good_poly_coeff[2]);
    float x2 = (-1 * (good_poly_coeff[1] - a) + sqrtf(delta)) / (2 * good_poly_coeff[2]);
    float y1 = a * x1 + b;
    float y2 = a * x2 + b;
    float dst1 = getDistance(cv::Point2f(x1, y1), line[line.size() / 2]);
    float dst2 = getDistance(cv::Point2f(x2, y2), line[line.size() / 2]);

    // check if nan
    if (dst1 != dst1)
      dst1 = right_lane_width_;
    if (dst2 != dst2)
      dst2 = right_lane_width_;

    width = std::min(dst1, dst2);
  }
  else
    width = std::abs(line[1].y - getAproxY(good_poly_coeff, line[1].x));
  if (!left_offset)
    width *= -1;

  if (polyfit(poly_nDegree_, createOffsetLine(good_poly_coeff, width), coeff))
    return coeff;
  else
  {
    coeff = good_poly_coeff;
    good_poly_coeff[0] -= width;
  }
}

void LaneDetector::calcRoadWidth()
{
  std::vector<cv::Point2f> center_line_points;

  cv::Point2f p;
  float increment = 0.05;
  for (float x = TOPVIEW_MIN_X; x < TOPVIEW_MAX_X; x += increment)
  {
    p.x = x;
    p.y = getAproxY(middle_coeff_, x);
    center_line_points.push_back(p);
  }

  float widthSum_r = 0;
  float widthSum_l = 0;
  float a_param_orthg = 0;
  float b_param_orthg = 0;
  float delta_right = 0, delta_left = 0;
  float x1_r = 0, x2_r = 0, y1_r = 0, y2_r = 0;
  float dst1_r = 0, dst2_r = 0;
  float x1_l = 0, x2_l = 0, y1_l = 0, y2_l = 0;
  float dst1_l = 0, dst2_l = 0;

  for (int i = 0; i < center_line_points.size(); i++)
  {
    a_param_orthg = -1 / (2 * last_middle_coeff_[2] * center_line_points[i].x + last_middle_coeff_[1]);
    // if full horizontal there are some errors in delta
    if (std::abs(a_param_orthg) > 200)
      a_param_orthg = 200;
    b_param_orthg = center_line_points[i].y - a_param_orthg * center_line_points[i].x;

    delta_right = ((last_right_coeff_[1] - a_param_orthg) * (last_right_coeff_[1] - a_param_orthg))
                  - 4 * (last_right_coeff_[2] * (last_right_coeff_[0] - b_param_orthg));
    delta_left = ((last_left_coeff_[1] - a_param_orthg) * (last_left_coeff_[1] - a_param_orthg))
                  - 4 * (last_left_coeff_[2] * (last_left_coeff_[0] - b_param_orthg));

    x1_r = (-1 * (last_right_coeff_[1] - a_param_orthg) - sqrtf(delta_right)) / (2 * last_right_coeff_[2]);
    x2_r = (-1 * (last_right_coeff_[1] - a_param_orthg) + sqrtf(delta_right)) / (2 * last_right_coeff_[2]);
    y1_r = a_param_orthg * x1_r + b_param_orthg;
    y2_r = a_param_orthg * x2_r + b_param_orthg;

    x1_l = (-1 * (last_left_coeff_[1] - a_param_orthg) - sqrtf(delta_left)) / (2 * last_left_coeff_[2]);
    x2_l = (-1 * (last_left_coeff_[1] - a_param_orthg) + sqrtf(delta_left)) / (2 * last_left_coeff_[2]);
    y1_l = a_param_orthg * x1_l + b_param_orthg;
    y2_l = a_param_orthg * x2_l + b_param_orthg;

    dst1_r = sqrtf(((x1_r - center_line_points[i].x) * (x1_r - center_line_points[i].x))
            + ((y1_r - center_line_points[i].y) * (y1_r - center_line_points[i].y)));
    dst2_r = sqrtf(((x2_r - center_line_points[i].x) * (x2_r - center_line_points[i].x))
            + ((y2_r - center_line_points[i].y) * (y2_r - center_line_points[i].y)));

    dst1_l = sqrtf(((x1_l - center_line_points[i].x) * (x1_l - center_line_points[i].x))
            + ((y1_l - center_line_points[i].y) * (y1_l - center_line_points[i].y)));
    dst2_l = sqrtf(((x2_l - center_line_points[i].x) * (x2_l - center_line_points[i].x))
            + ((y2_l - center_line_points[i].y) * (y2_l - center_line_points[i].y)));

    if (dst1_r > dst2_r)
      widthSum_r += dst2_r;
    else
      widthSum_r += dst1_r;

    if (dst1_l > dst2_l)
      widthSum_l += dst2_l;
    else
      widthSum_l += dst1_l;
  }

  if (widthSum_r / center_line_points.size() < 0.47 && widthSum_r / center_line_points.size() > 0.35)
    right_lane_width_ = widthSum_r / center_line_points.size();

  if (widthSum_l / center_line_points.size() < 0.47 && widthSum_l / center_line_points.size() > 0.35)
    left_lane_width_ = widthSum_l / center_line_points.size();
}

void LaneDetector::generatePoints()
{
  if (left_line_index_ != -1)
  {
    if (lanes_vector_converted_[left_line_index_].size()
        / cv::arcLength(lanes_vector_converted_[left_line_index_], false) < points_density_ * 2)
    {
      for (int i = 0; i < lanes_vector_converted_[left_line_index_].size() - 1; i++)
      {
        float distance = getDistance(lanes_vector_converted_[left_line_index_][i],
                                     lanes_vector_converted_[left_line_index_][i + 1]);
        if (distance > 1 / points_density_)
        {
          int add = distance * points_density_;
          cv::Point2f p;
          float x1 = lanes_vector_converted_[left_line_index_][i].x;
          float y1 = lanes_vector_converted_[left_line_index_][i].y;
          float x_dif = (lanes_vector_converted_[left_line_index_][i + 1].x
                        - lanes_vector_converted_[left_line_index_][i].x) / (add + 1);
          float y_dif = (lanes_vector_converted_[left_line_index_][i + 1].y
                        - lanes_vector_converted_[left_line_index_][i].y) / (add + 1);
          for (int j = 0; j < add; j++)
          {
            p.x = x1 + x_dif * (j + 1);
            p.y = y1 + y_dif * (j + 1);
            lanes_vector_converted_[left_line_index_].insert(lanes_vector_converted_[left_line_index_].begin()
                                                            + i + 1, p);
            i++;
          }
        }
      }
    }
  }

  if (center_line_index_ != -1)
  {
    if (lanes_vector_converted_[center_line_index_].size()
        / cv::arcLength(lanes_vector_converted_[center_line_index_], false) < points_density_ * 2)
    {
      for (int i = 0; i < lanes_vector_converted_[center_line_index_].size() - 1; i++)
      {
        float distance = getDistance(lanes_vector_converted_[center_line_index_][i],
                                     lanes_vector_converted_[center_line_index_][i + 1]);
        if (distance > 1 / points_density_)
        {
          int add = distance * points_density_;
          cv::Point2f p;
          float x1 = lanes_vector_converted_[center_line_index_][i].x;
          float y1 = lanes_vector_converted_[center_line_index_][i].y;
          float x_dif = (lanes_vector_converted_[center_line_index_][i + 1].x
                        - lanes_vector_converted_[center_line_index_][i].x) / (add + 1);
          float y_dif = (lanes_vector_converted_[center_line_index_][i + 1].y
                        - lanes_vector_converted_[center_line_index_][i].y) / (add + 1);
          for (int j = 0; j < add; j++)
          {
            p.x = x1 + x_dif * (j + 1);
            p.y = y1 + y_dif * (j + 1);
            lanes_vector_converted_[center_line_index_].insert(lanes_vector_converted_[center_line_index_].begin()
                                                              + i + 1, p);
            i++;
          }
        }
      }
    }
  }

  if (right_line_index_ != -1)
  {
    if (lanes_vector_converted_[right_line_index_].size()
        / cv::arcLength(lanes_vector_converted_[right_line_index_], false) < points_density_ * 2)
    {
      for (int i = 0; i < lanes_vector_converted_[right_line_index_].size() - 1; i++)
      {
        float distance = getDistance(lanes_vector_converted_[right_line_index_][i],
                                     lanes_vector_converted_[right_line_index_][i + 1]);
        if (distance > 1 / points_density_)
        {
          int add = distance * points_density_;
          cv::Point2f p;
          float x1 = lanes_vector_converted_[right_line_index_][i].x;
          float y1 = lanes_vector_converted_[right_line_index_][i].y;
          float x_dif = (lanes_vector_converted_[right_line_index_][i + 1].x
                        - lanes_vector_converted_[right_line_index_][i].x) / (add + 1);
          float y_dif = (lanes_vector_converted_[right_line_index_][i + 1].y
                        - lanes_vector_converted_[right_line_index_][i].y) / (add + 1);
          for (int j = 0; j < add; j++)
          {
            p.x = x1 + x_dif * (j + 1);
            p.y = y1 + y_dif * (j + 1);
            lanes_vector_converted_[right_line_index_].insert(lanes_vector_converted_[right_line_index_].begin()
                                                              + i + 1, p);
            i++;
          }
        }
      }
    }
  }
}

void LaneDetector::removeHorizontalLines()
{
  for (int i = 0; i < lanes_vector_converted_.size(); i++)
  {
    float dx = lanes_vector_converted_[i][lanes_vector_converted_[i].size() - 1].x - lanes_vector_converted_[i][0].x;
    if (dx < 0.1)
      lanes_vector_converted_.erase(lanes_vector_converted_.begin() + i);
  }
}

std::vector<cv::Point2f> LaneDetector::createOffsetLine(std::vector<float> coeff, float offset)
{
  std::vector<cv::Point2f> new_line;
  cv::Point2f p;
  for (float i = TOPVIEW_MIN_X - 0.1; i < TOPVIEW_MAX_X + 0.1; i += 0.05)
  {
    float deriative = 2 * coeff[2] * i + coeff[1];
    float angle = atan(deriative);
    p.x = i - offset * sin(angle);
    p.y = getAproxY(coeff, i) + offset * cos(angle);
    new_line.push_back(p);
  }
  return new_line;
}

void LaneDetector::detectStartAndIntersectionLine()
{
  float detect_slope = 2.2;
  float detect_lenght = 0.2;
  // morphology variables
  int morph_size = 7;
  int morph_elem = 0;

  // vectors
  std::vector<std::vector<cv::Point>> left_lines;
  std::vector<std::vector<cv::Point>> right_lines;
  std::vector<std::vector<cv::Point2f>> left_lines_converted;
  std::vector<std::vector<cv::Point2f>> right_lines_converted;

  float right_distance = -1000;
  float left_distance = -1000;

  int operation = 3;
  cv::Mat element = getStructuringElement(morph_elem, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                          cv::Point(morph_size, morph_size));
  cv::morphologyEx(left_lane_frame_, left_lane_frame_, operation, element);
  cv::morphologyEx(right_lane_frame_, right_lane_frame_, operation, element);
  cv::medianBlur(left_lane_frame_, left_lane_frame_, 5);
  cv::medianBlur(right_lane_frame_, right_lane_frame_, 5);

  detectLines(left_lane_frame_, left_lines);
  detectLines(right_lane_frame_, right_lines);

  // check right lane
  for (int i = 0; i < right_lines.size(); i++)
  {
    cv::RotatedRect rect = cv::minAreaRect(right_lines[i]);

    cv::Point2f vertices2f[4];
    rect.points(vertices2f);

    std::vector<cv::Point2f> rect_vector;
    rect_vector.push_back(vertices2f[0]);
    rect_vector.push_back(vertices2f[1]);
    rect_vector.push_back(vertices2f[2]);
    rect_vector.push_back(vertices2f[3]);
    cv::transform(rect_vector, rect_vector, topview2world_.rowRange(0, 2));

    float d1, d2;
    d1 = getDistance(rect_vector[0], rect_vector[1]);
    d2 = getDistance(rect_vector[1], rect_vector[2]);
    if (d1 < detect_lenght && d2 < detect_lenght)
      continue;
    float a = 0;
    if (d1 > d2)
    {
      if (rect_vector[0].x - rect_vector[1].x != 0)
        a = (rect_vector[0].y - rect_vector[1].y) / (rect_vector[0].x - rect_vector[1].x);
      else
        a = 100;
    }
    else
    {
      if (rect_vector[1].x - rect_vector[2].x != 0)
        a = (rect_vector[1].y - rect_vector[2].y) / (rect_vector[1].x - rect_vector[2].x);
      else
        a = 100;
    }

    if (std::abs(a) > detect_slope)
    {
      right_distance = (rect_vector[0].x + rect_vector[1].x + rect_vector[2].x + rect_vector[3].x) / 4;
      break;
    }
  }

  // check left lane
  for (int i = 0; i < left_lines.size(); i++)
  {
    cv::RotatedRect rect = cv::minAreaRect(left_lines[i]);

    cv::Point2f vertices2f[4];
    rect.points(vertices2f);

    std::vector<cv::Point2f> rect_vector;
    rect_vector.push_back(vertices2f[0]);
    rect_vector.push_back(vertices2f[1]);
    rect_vector.push_back(vertices2f[2]);
    rect_vector.push_back(vertices2f[3]);
    cv::transform(rect_vector, rect_vector, topview2world_.rowRange(0, 2));

    float d1, d2;
    d1 = getDistance(rect_vector[0], rect_vector[1]);
    d2 = getDistance(rect_vector[1], rect_vector[2]);
    if (d1 < detect_lenght && d2 < detect_lenght)
      continue;
    float a = 0;
    if (d1 > d2)
    {
      if (rect_vector[0].x - rect_vector[1].x != 0)
        a = (rect_vector[0].y - rect_vector[1].y) / (rect_vector[0].x - rect_vector[1].x);
      else
        a = 100;
    }
    else
    {
      if (rect_vector[1].x - rect_vector[2].x != 0)
        a = (rect_vector[1].y - rect_vector[2].y) / (rect_vector[1].x - rect_vector[2].x);
      else
        a = 100;
    }

    if (std::abs(a) > detect_slope)
    {
      left_distance = (rect_vector[0].x + rect_vector[1].x + rect_vector[2].x + rect_vector[3].x) / 4;
      break;
    }
  }

  if (std::abs(right_distance - left_distance) < 0.09 && left_distance > 0)
  {
    std_msgs::Float32 msg;
    msg.data = right_distance;
    starting_line_pub_.publish(msg);
  }
  else if (right_distance > 0)
  {
    std_msgs::Float32 msg;
    msg.data = right_distance;
    intersection_pub_.publish(msg);
  }
}
