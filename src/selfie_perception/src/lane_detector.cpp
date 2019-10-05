/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <selfie_perception/lane_detector.h>
#include <vector>
#include <algorithm>

#define TOPVIEW_ROWS 172
#define TOPVIEW_COLS 640

#define TOPVIEW_MIN_X 0.3
#define TOPVIEW_MAX_X 1.0
#define TOPVIEW_MIN_Y -1.3
#define TOPVIEW_MAX_Y 1.3

static double Acc_value = 1.0;
static int Acc_filt = 5;

LaneDetector::LaneDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
  nh_(nh),
  pnh_(pnh),
  it_(nh)
{
  lanes_pub_ = nh_.advertise<selfie_msgs::RoadMarkings>("road_markings", 100);
  intersection_pub_ = nh_.advertise<std_msgs::Float32>("intersection", 100);
  starting_line_pub_ = nh_.advertise<std_msgs::Float32>("starting_line", 100);
}

LaneDetector::~LaneDetector()
{
  cv::destroyAllWindows();
}

bool LaneDetector::init()
{
  lines_vector_.clear();
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

  getParams();

  treshold_block_size_ = static_cast<int>(TOPVIEW_COLS / (TOPVIEW_MAX_Y - TOPVIEW_MIN_Y) * real_window_size_);
  if (treshold_block_size_ % 2 == 0)
    treshold_block_size_++;

  // calc hom_cut_mask
  hom_cut_mask_ = cv::Mat::zeros(cv::Size(TOPVIEW_COLS, TOPVIEW_ROWS), CV_8UC1);
  if (hom_cut_r_x_ != 0 && hom_cut_r_y_ != 0)
  {
    cv::Point points[3];
    points[0] = cv::Point(hom_cut_r_x_, homography_frame_.rows);
    points[1] = cv::Point(homography_frame_.cols, homography_frame_.rows);
    points[2] = cv::Point(homography_frame_.cols, hom_cut_r_y_);
    cv::fillConvexPoly(hom_cut_mask_, points, 3, cv::Scalar(255, 255, 255));
    points[0] = cv::Point(hom_cut_l_x_, homography_frame_.rows);
    points[1] = cv::Point(0, homography_frame_.rows);
    points[2] = cv::Point(0 , hom_cut_l_y_);
    cv::fillConvexPoly(hom_cut_mask_, points, 3, cv::Scalar(255, 255, 255));
  }
  cv::bitwise_not(hom_cut_mask_, hom_cut_mask_);

  image_sub_ = it_.subscribe("/image_rect", 10, &LaneDetector::imageCallback, this);
  if (debug_mode_)
  {
    points_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("new_coordinates", 10);
    aprox_visualization_pub_ = nh_.advertise<visualization_msgs::Marker>("aprox", 10);
  }

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

  cv::adaptiveThreshold(homography_frame_, binary_frame_, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                        CV_THRESH_BINARY, treshold_block_size_, threshold_c_);

  cv::bitwise_and(binary_frame_, hom_cut_mask_, binary_cut_frame_);
  cv::medianBlur(binary_cut_frame_, binary_cut_frame_, 3);

  if (!init_imageCallback_)
  {
    dynamicMask(binary_cut_frame_, masked_frame_);
    if (debug_mode_)
      cv::bitwise_and(homography_frame_, dynamic_mask_, homography_frame_);

    // remove ROI inside left and right lane
    ROILaneRight(masked_frame_, masked_frame_);
    ROILaneLeft(masked_frame_, masked_frame_);
    detectStartAndIntersectionLine();
  }
  else
  {
    cv::filter2D(binary_cut_frame_, masked_frame_, -1, kernel_v_, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  }
  
  detectLines(masked_frame_, lines_vector_);
  if (lines_vector_.empty())
  {
    left_line_.index = -1;
    center_line_.index = -1;
    right_line_.index = -1;
    return;
  }
  convertCoordinates();
  filterSmallLines();
  if (lines_vector_converted_.empty())
  {
    left_line_.index = -1;
    center_line_.index = -1;
    right_line_.index = -1;
    return;
  }

  mergeMiddleLines();

  if (init_imageCallback_)
  {
    initRecognizeLines();
    calcRoadLinesParams();
    int l = 0, c = 0, r = 0;
    if (!left_line_.is_short)
      l = 1;

    if (!center_line_.is_short)
      c = 1;

    if (!right_line_.is_short)
      r = 1;

    if ((l + r + c) > 1)
    {
      linesApproximation();
      init_imageCallback_ = false;
    }
  }
  else
  {
    recognizeLines();
    generatePoints();
    addBottomPoint();
    calcRoadLinesParams();

    if (center_line_.index != -1)
      calcRoadWidth();
    linesApproximation();

    publishMarkings();
  }

  if (debug_mode_)
  {
    debug_frame_.rows = homography_frame_.rows;
    debug_frame_.cols = homography_frame_.cols;
    lanesVectorVisualization(debug_frame_);

    convertApproxToFrameCoordinate();
    drawAproxOnHomography();
    openCVVisualization();
    aproxRVIZVisualization();
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
  for (int i = 0; i < output_lanes.size(); ++i)
  {
    cv::approxPolyDP(cv::Mat(output_lanes[i]), output_lanes[i], Acc_value, false);
  }
}

void LaneDetector::drawAproxOnHomography()
{
  cv::cvtColor(homography_frame_, homography_frame_, CV_GRAY2BGR);
  if (!aprox_lines_frame_coordinate_[2].empty())
    for (int i = 0; i < aprox_lines_frame_coordinate_[2].size(); ++i)
    {
      cv::circle(homography_frame_, aprox_lines_frame_coordinate_[2][i], 2, cv::Scalar(255, 0, 0), CV_FILLED);
    }
  if (!aprox_lines_frame_coordinate_[0].empty())
    for (int i = 0; i < aprox_lines_frame_coordinate_[0].size(); ++i)
    {
      cv::circle(homography_frame_, aprox_lines_frame_coordinate_[0][i], 2, cv::Scalar(0, 0, 255), CV_FILLED);
    }
  if (!aprox_lines_frame_coordinate_[1].empty())
    for (int i = 0; i < aprox_lines_frame_coordinate_[1].size(); ++i)
    {
      cv::circle(homography_frame_, aprox_lines_frame_coordinate_[1][i], 2, cv::Scalar(0, 255, 0), CV_FILLED);
    }
}

void LaneDetector::homography(cv::Mat input_frame, cv::Mat &homography_frame)
{
  cv::warpPerspective(input_frame, homography_frame, topview2cam_,
                      topview_size_, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
}

void LaneDetector::getParams()
{
  cv::FileStorage fs;
  if (pnh_.getParam("config_file", config_file_) && fs.open(config_file_, cv::FileStorage::READ))
  {
    fs["world2cam"] >> world2cam_;
    fs.release();
  }
  else
  {
    // default homography matrix
    ROS_INFO("Default homography matrix");
    world2cam_ = 
    (cv::Mat_<double>(3,3) << 
    -2866.861836770729,   1637.56986272477,     -421.4096359156129,
    -2206.631043183711,   -117.8258687217328,   -738.93303219503,
    -4.411887214969603,   -0.1088704319653951,  -0.5957339297068464);
  }
  
  pnh_.getParam("real_window_size", real_window_size_);
  pnh_.getParam("threshold_c", threshold_c_);
  pnh_.getParam("debug_mode", debug_mode_);
  pnh_.getParam("hom_cut_tune_mode", hom_cut_tune_mode_);
  pnh_.getParam("max_mid_line_gap", max_mid_line_gap_);

  pnh_.getParam("hom_cut_l_x", hom_cut_l_x_);
  pnh_.getParam("hom_cut_l_y", hom_cut_l_y_);
  pnh_.getParam("hom_cut_r_x", hom_cut_r_x_);
  pnh_.getParam("hom_cut_r_y", hom_cut_r_y_);
}

void LaneDetector::openCVVisualization()
{
  cv::namedWindow("STD_DEBUG", cv::WINDOW_NORMAL);
  cv::Mat m_binary, m_std, m_four;
  cv::hconcat(binary_cut_frame_, masked_frame_, m_binary);
  cv::cvtColor(m_binary, m_binary, CV_GRAY2BGR);
  cv::hconcat(debug_frame_, homography_frame_, m_std);
  cv::vconcat(m_binary, m_std, m_four);

  cv::imshow("STD_DEBUG", m_four);

  if (hom_cut_tune_mode_)
  {
    cv::namedWindow("binary_frame", cv::WINDOW_NORMAL);
    cv::imshow("binary_frame", binary_frame_);

    cv::namedWindow("binary_cut_frame", cv::WINDOW_NORMAL);
    cv::imshow("binary_cut_frame", binary_cut_frame_);
  }
  cv::waitKey(1);
}

void LaneDetector::quickSortLinesY(int left, int right)
{
  int i = left;
  int j = right;
  int y = lines_vector_[(left + right) / 2][0].y;
  do
  {
    while (lines_vector_[i][0].y > y)
      ++i;

    while (lines_vector_[j][0].y < y)
      j--;

    if (i <= j)
    {
      lines_vector_[i].swap(lines_vector_[j]);

      ++i;
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
      ++i;

    while (vector_in[j].y < y)
      j--;

    if (i <= j)
    {
      cv::Point temp = vector_in[i];
      vector_in[i] = vector_in[j];
      vector_in[j] = temp;

      ++i;
      j--;
    }
  }
  while (i <= j);

  if (left < j)
    LaneDetector::quickSortPointsY(vector_in, left, j);

  if (right > i)
    LaneDetector::quickSortPointsY(vector_in, i, right);
}

void LaneDetector::mergeMiddleLines()
{
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    float a, b;
    for (int j = i + 1; j < lines_vector_converted_.size(); ++j)
    {
      if (lines_vector_converted_[i][lines_vector_converted_[i].size() - 1].x - lines_vector_converted_[i][0].x != 0)
        a = (lines_vector_converted_[i][0].y - lines_vector_converted_[i][lines_vector_converted_[i].size() - 1].y)
            / (lines_vector_converted_[i][0].x - lines_vector_converted_[i][lines_vector_converted_[i].size() - 1].x);
      else
        a = 999999;
      b = lines_vector_converted_[i][0].y - a * lines_vector_converted_[i][0].x;

      float distance = std::abs(a * lines_vector_converted_[j][0].x - lines_vector_converted_[j][0].y + b)
                       / sqrtf(a * a + 1);
      float distance2 = getDistance(lines_vector_converted_[j][0],
                                    lines_vector_converted_[i][lines_vector_converted_[i].size() - 1]);

      if (distance < max_mid_line_distance_ && distance2 < max_mid_line_gap_)
      {
        lines_vector_converted_[i].insert(lines_vector_converted_[i].end(),
                                          lines_vector_converted_[j].begin(), lines_vector_converted_[j].end());
        lines_vector_converted_.erase(lines_vector_converted_.begin() + j);
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
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    float aprox_y = getAproxY(left_line_.coeff, lines_vector_converted_[i][0].x);
    if (std::abs(aprox_y - lines_vector_converted_[i][0].y) < max_delta_y_lane_)
    {
      closest_container.push_back(lines_vector_converted_[i]);
      closest_index.push_back(i);
    }
  }
  if (closest_container.empty())
    left_line_.index = -1;
  else
  {
    float aprox_y = getAproxY(left_line_.coeff, closest_container[0][0].x);
    float min_sum_distance = std::abs(aprox_y - closest_container[0][0].y);
    aprox_y = getAproxY(left_line_.coeff, closest_container[0][closest_container[0].size() / 2].x);
    min_sum_distance += std::abs(aprox_y - closest_container[0][closest_container[0].size() / 2].y);
    left_line_.index = closest_index[0];

    for (int i = 1; i < closest_container.size(); ++i)
    {
      aprox_y = getAproxY(left_line_.coeff, closest_container[i][0].x);
      float sum_distance = std::abs(aprox_y - closest_container[i][0].y);
      aprox_y = getAproxY(left_line_.coeff, closest_container[i][closest_container[i].size() / 2].x);
      sum_distance += std::abs(aprox_y - closest_container[i][closest_container[i].size() / 2].y);
      if (sum_distance < min_sum_distance)
      {
        left_line_.index = closest_index[i];
        min_sum_distance = sum_distance;
      }
    }
  }
  closest_container.clear();
  closest_index.clear();

  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    if (i == left_line_.index)
      continue;
    float aprox_y = getAproxY(right_line_.coeff, lines_vector_converted_[i][0].x);
    if (std::abs(aprox_y - lines_vector_converted_[i][0].y) < max_delta_y_lane_)
    {
      closest_container.push_back(lines_vector_converted_[i]);
      closest_index.push_back(i);
    }
  }
  if (closest_container.empty())
    right_line_.index = -1;
  else
  {
    float aprox_y = getAproxY(right_line_.coeff, closest_container[0][0].x);
    float min_sum_distance = std::abs(aprox_y - closest_container[0][0].y);
    aprox_y = getAproxY(right_line_.coeff, closest_container[0][closest_container[0].size() / 2].x);
    min_sum_distance += std::abs(aprox_y - closest_container[0][closest_container[0].size() / 2].y);
    right_line_.index = closest_index[0];

    for (int i = 1; i < closest_container.size(); ++i)
    {
      aprox_y = getAproxY(right_line_.coeff, closest_container[i][0].x);
      float sum_distance = std::abs(aprox_y - closest_container[i][0].y);
      aprox_y = getAproxY(right_line_.coeff, closest_container[i][closest_container[i].size() / 2].x);
      sum_distance += std::abs(aprox_y - closest_container[i][closest_container[i].size() / 2].y);
      if (sum_distance < min_sum_distance)
      {
        right_line_.index = closest_index[i];
        min_sum_distance = sum_distance;
      }
    }
  }
  closest_container.clear();
  closest_index.clear();

  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    if (i == left_line_.index || i == right_line_.index)
      continue;
    float aprox_y = getAproxY(center_line_.coeff, lines_vector_converted_[i][0].x);
    if (std::abs(aprox_y - lines_vector_converted_[i][0].y) < max_delta_y_lane_)
    {
      closest_container.push_back(lines_vector_converted_[i]);
      closest_index.push_back(i);
    }
  }
  if (closest_container.empty())
    center_line_.index = -1;
  else
  {
    float aprox_y = getAproxY(center_line_.coeff, closest_container[0][0].x);
    float min_sum_distance = std::abs(aprox_y - closest_container[0][0].y);
    aprox_y = getAproxY(center_line_.coeff, closest_container[0][closest_container[0].size() / 2].x);
    min_sum_distance += std::abs(aprox_y - closest_container[0][closest_container[0].size() / 2].y);
    center_line_.index = closest_index[0];

    for (int i = 1; i < closest_container.size(); ++i)
    {
      aprox_y = getAproxY(center_line_.coeff, closest_container[i][0].x);
      float sum_distance = std::abs(aprox_y - closest_container[i][0].y);
      aprox_y = getAproxY(center_line_.coeff, closest_container[i][closest_container[i].size() / 2].x);
      sum_distance += std::abs(aprox_y - closest_container[i][closest_container[i].size() / 2].y);
      if (sum_distance < min_sum_distance)
      {
        center_line_.index = closest_index[i];
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
  for (int i = 0; i < left_line_.coeff.size(); ++i)
  {
    road_markings.left_line.push_back(left_line_.coeff[i]);
    road_markings.right_line.push_back(right_line_.coeff[i]);
    road_markings.center_line.push_back(center_line_.coeff[i]);
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
  if (right_line_.index == -1)
    offset_right = -0.14;
  else if (lines_vector_converted_[right_line_.index][lines_vector_converted_[right_line_.index].size() - 1].x
            < ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 4))
    offset_right = -0.14;
  if (left_line_.index == -1)
    offset_left = 0.12;
  else if (lines_vector_converted_[left_line_.index][lines_vector_converted_[left_line_.index].size() - 1].x
            < ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 4))
    offset_left = 0.12;

  std::vector<cv::Point2f> left_line_offset = createOffsetLine(left_line_, offset_left);
  std::vector<cv::Point2f> right_line_offset = createOffsetLine(right_line_, offset_right);
  cv::transform(left_line_offset, left_line_offset, world2topview_.rowRange(0, 2));
  cv::transform(right_line_offset, right_line_offset, world2topview_.rowRange(0, 2));

  int l, r;
  const int KLength = left_line_offset.size() + right_line_offset.size();
  cv::Point points[KLength];
  for (l = 0; l < left_line_offset.size(); ++l)
  {
    points[l] = cv::Point(left_line_offset[l].x, left_line_offset[l].y);
  }
  for (r = right_line_offset.size() - 1; r >= 0; r--)
  {
    points[l] = cv::Point(right_line_offset[r].x, right_line_offset[r].y);
    ++l;
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
  if (starting_line_)
  {
    offset_right = 0.02;
    offset_center = -0.03;
  }
  else
  {
    if (right_line_.index == -1 || right_line_.is_short)
      offset_right = 0.11;
    if (center_line_.index == -1 || center_line_.is_short)
      offset_center = -0.13;
  }
  
  

  std::vector<cv::Point2f> center_line_offset = createOffsetLine(center_line_, offset_center);
  std::vector<cv::Point2f> right_line_offset = createOffsetLine(right_line_, offset_right);
  cv::transform(center_line_offset, center_line_offset, world2topview_.rowRange(0, 2));
  cv::transform(right_line_offset, right_line_offset, world2topview_.rowRange(0, 2));

  int c, r;
  const int KLength = center_line_offset.size() + right_line_offset.size();
  cv::Point points[KLength];
  for (c = 0; c < center_line_offset.size(); ++c)
  {
    points[c] = cv::Point(center_line_offset[c].x, center_line_offset[c].y);
  }
  for (r = right_line_offset.size() - 1; r >= 0; r--)
  {
    points[c] = cv::Point(right_line_offset[r].x, right_line_offset[r].y);
    ++c;
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
  if (starting_line_)
  {
    offset_center = 0.02;
    offset_left = -0.03;
  }
  else
  {
    if (left_line_.index == -1 || left_line_.is_short)
      offset_left = -0.13;
    if (center_line_.index == -1 || center_line_.is_short)
      offset_center = 0.11;
  }
  
  std::vector<cv::Point2f> center_line_offset = createOffsetLine(center_line_, offset_center);
  std::vector<cv::Point2f> left_line_offset = createOffsetLine(left_line_, offset_left);
  cv::transform(center_line_offset, center_line_offset, world2topview_.rowRange(0, 2));
  cv::transform(left_line_offset, left_line_offset, world2topview_.rowRange(0, 2));

  int c, l;
  const int KLength = center_line_offset.size() + left_line_offset.size();
  cv::Point points[KLength];
  for (c = 0; c < center_line_offset.size(); ++c)
  {
    points[c] = cv::Point(center_line_offset[c].x, center_line_offset[c].y);
  }
  for (l = left_line_offset.size() - 1; l >= 0; l--)
  {
    points[c] = cv::Point(left_line_offset[l].x, left_line_offset[l].y);
    ++c;
  }

  cv::fillConvexPoly(left_lane_ROI_, points, KLength, cv::Scalar(255, 255, 255));

  left_lane_frame_ = input_frame.clone();
  cv::bitwise_and(input_frame, left_lane_ROI_, left_lane_frame_);

  cv::bitwise_not(left_lane_ROI_, left_lane_ROI_);
  cv::bitwise_and(input_frame, left_lane_ROI_, output_frame);
}

void LaneDetector::filterSmallLines()
{
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    float sum = arcLength(lines_vector_converted_[i], false);
    if (sum < min_length_search_line_)
    {
      lines_vector_converted_.erase(lines_vector_converted_.begin() + i);
      i--;
    }
  }
}

void LaneDetector::convertCoordinates()
{
  for (int i = 0; i < lines_vector_.size(); ++i)
  {
    quickSortPointsY(lines_vector_[i], 0, lines_vector_[i].size() - 1);
  }
  quickSortLinesY(0, lines_vector_.size() - 1);

  lines_vector_converted_.clear();
  for (int i = 0; i < lines_vector_.size(); ++i)
  {
    std::vector<cv::Point2f> line;
    cv::Mat(lines_vector_[i]).copyTo(line);

    cv::transform(line, line, topview2world_.rowRange(0, 2));

    lines_vector_converted_.push_back(line);
  }
}

float LaneDetector::getAproxY(std::vector<float> coeff, float x)
{
  size_t nDegree = coeff.size();

  float nY = 0;
  float nXdouble = 1;
  float nX = x;

  for (size_t j = 0; j < nDegree; ++j)
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
  float increment = 0.04;
  for (float x = TOPVIEW_MIN_X; x < TOPVIEW_MAX_X; x += increment)
  {
    p.x = x;

    p.y = getAproxY(left_line_.coeff, x);
    aprox_lines_frame_coordinate_[0].push_back(p);

    p.y = getAproxY(center_line_.coeff, x);
    aprox_lines_frame_coordinate_[1].push_back(p);

    p.y = getAproxY(right_line_.coeff, x);
    aprox_lines_frame_coordinate_[2].push_back(p);
  }
  cv::transform(aprox_lines_frame_coordinate_[0], aprox_lines_frame_coordinate_[0], world2topview_.rowRange(0, 2));
  cv::transform(aprox_lines_frame_coordinate_[1], aprox_lines_frame_coordinate_[1], world2topview_.rowRange(0, 2));
  cv::transform(aprox_lines_frame_coordinate_[2], aprox_lines_frame_coordinate_[2], world2topview_.rowRange(0, 2));
}

void LaneDetector::initRecognizeLines()
{
  // check lines growth
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    for (int j = 1; j < lines_vector_converted_[i].size(); ++j)
    {
      if (lines_vector_converted_[i][j - 1].x - lines_vector_converted_[i][j].x > 0)
      {
        lines_vector_converted_[i].erase(lines_vector_converted_[i].begin() + j, lines_vector_converted_[i].end());
        break;
      }
    }
  }

  float min = homography_frame_.cols;
  int min_index = -1;
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    if (std::abs(nominal_center_line_Y_ - lines_vector_converted_[i][0].y) < min && lines_vector_converted_[i][0].x
        < 1 && cv::arcLength(lines_vector_converted_[i], false) > 0.15)
    {
      min = std::abs(nominal_center_line_Y_ - lines_vector_converted_[i][0].y);
      min_index = i;
    }
  }
  center_line_.index = min_index;

  if (center_line_.index == -1)
  {
    right_line_.index = -1;
    left_line_.index = -1;
    return;
  }

  min = homography_frame_.cols;
  min_index = -1;
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    if (lines_vector_converted_[i][0].y < lines_vector_converted_[center_line_.index][0].y &&
        lines_vector_converted_[i][0].x < 1 && cv::arcLength(lines_vector_converted_[i], false) > 0.2)
    {
      float distance_last = getDistance(lines_vector_converted_[center_line_.index][0], lines_vector_converted_[i][0]);
      float distance;
      for (int j = 1; j < lines_vector_converted_[i].size(); ++j)
      {
        distance = getDistance(lines_vector_converted_[center_line_.index][0], lines_vector_converted_[i][j]);
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
  right_line_.index = min_index;

  min = homography_frame_.cols;
  min_index = -1;
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    if (lines_vector_converted_[i][0].y > lines_vector_converted_[center_line_.index][0].y &&
        lines_vector_converted_[i][0].x < 1 && cv::arcLength(lines_vector_converted_[i], false) > 0.2)
    {
      float distance_last = getDistance(lines_vector_converted_[center_line_.index][0], lines_vector_converted_[i][0]);
      float distance;
      for (int j = 1; j < lines_vector_converted_[i].size(); ++j)
      {
        distance = getDistance(lines_vector_converted_[center_line_.index][0], lines_vector_converted_[i][j]);
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
  left_line_.index = min_index;
}

void LaneDetector::pointsRVIZVisualization()
{
  geometry_msgs::Point32 point;
  point.z = 0;
  points_cloud_.points.clear();
  points_cloud_.header.frame_id = "road_markings";

  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    for (int j = 0; j < lines_vector_converted_[i].size(); ++j)
    {
      point.x = lines_vector_converted_[i][j].x;
      point.y = lines_vector_converted_[i][j].y;
      points_cloud_.points.push_back(point);
    }
  }

  points_cloud_pub_.publish(points_cloud_);
}

void LaneDetector::aproxRVIZVisualization()
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
    marker_point.y = getAproxY(left_line_.coeff, x);
    marker.points.push_back(marker_point);
  }
  for (float x = TOPVIEW_MIN_X; x < TOPVIEW_MAX_X; x += increment)
  {
    marker_point.x = x;
    marker_point.y = getAproxY(center_line_.coeff, x);
    marker.points.push_back(marker_point);
  }
  for (float x = TOPVIEW_MIN_X; x < TOPVIEW_MAX_X; x += increment)
  {
    marker_point.x = x;
    marker_point.y = getAproxY(right_line_.coeff, x);
    marker.points.push_back(marker_point);
  }
  aprox_visualization_pub_.publish(marker);
}

void LaneDetector::linesApproximation()
{
  unsigned char l_state, c_state, r_state;

  int suitable_lines = 3;
  if (left_line_.is_short)
    --suitable_lines;
  if (right_line_.is_short)
    --suitable_lines;
  if (center_line_.is_short)
    --suitable_lines;

  switch (suitable_lines)
  {
  case 3:
    // "l+  c+  r+"
    l_state = '+';
    c_state = '+';
    r_state = '+';

    polyfit(left_line_);
    polyfit(center_line_);
    polyfit(right_line_);
    break;

  case 2:
    if (left_line_.is_short)
    {
      polyfit(center_line_);
      polyfit(right_line_);

      if (left_line_.index == -1)
      {
        // "l-  c+  r+"
        l_state = '-';
        c_state = '+';
        r_state = '+';
        left_line_.degree = center_line_.degree;
        polyfit(createOffsetLine(center_line_, left_lane_width_), left_line_);
      }
      else
      {
        // l/  c+  r+"
        l_state = '/';
        c_state = '+';
        r_state = '+';
        adjust(center_line_, left_line_, true);
      }
    }
    else if (right_line_.is_short)
    {
      polyfit(left_line_);
      polyfit(center_line_);

      if (right_line_.index == -1)
      {
        // "l+  c+  r-"
        l_state = '+';
        c_state = '+';
        r_state = '-';
        right_line_.degree = center_line_.degree;
        polyfit(createOffsetLine(center_line_, -1 * right_lane_width_), right_line_);
      }
      else
      {
        // "l+  c+  r/"
        l_state = '+';
        c_state = '+';
        r_state = '/';
        adjust(center_line_, right_line_, false);
      }
    }
    else if (center_line_.is_short)
    {
      polyfit(left_line_);
      polyfit(right_line_);

      if (center_line_.index == -1)
      {
        // "l+  c-  r+"
        l_state = '+';
        c_state = '-';
        r_state = '+';
        center_line_.degree = right_line_.degree;
        polyfit(createOffsetLine(right_line_, right_lane_width_), center_line_);
      }
      else
      {
        // "l+  c/  r+"
        l_state = '+';
        c_state = '/';
        r_state = '+';
        adjust(right_line_, center_line_, true);
      }
    }
    break;

  case 1:
    if (!right_line_.is_short)
    {
      polyfit(right_line_);
      if (center_line_.index != -1 || left_line_.index != -1)
      {
        if (center_line_.index != -1 && left_line_.index != -1)
        {
          // "l/  c/  r+"
          l_state = '/';
          c_state = '/';
          r_state = '+';
          adjust(right_line_, center_line_, true);
          adjust(right_line_, left_line_, true);
        }
        else if (center_line_.index != -1)
        {
          // "l-  c/  r+"
          l_state = '-';
          c_state = '/';
          r_state = '+';
          adjust(right_line_, center_line_, true);
          left_line_.degree = center_line_.degree;
          polyfit(createOffsetLine(center_line_, left_lane_width_), left_line_);
        }
        else
        {
          // "l/  c-  r+"
          l_state = '/';
          c_state = '-';
          r_state = '+';
          adjust(right_line_, left_line_, true);
          center_line_.degree = right_line_.degree;
          polyfit(createOffsetLine(right_line_, right_lane_width_), center_line_);
        }
      }
      else
      {
        // "l-  c-  r+"
        l_state = '-';
        c_state = '-';
        r_state = '+';
        center_line_.degree = right_line_.degree;
        polyfit(createOffsetLine(right_line_, right_lane_width_), center_line_);
        left_line_.degree = center_line_.degree;
        polyfit(createOffsetLine(center_line_, left_lane_width_), left_line_);
      }
    }

    else if (!left_line_.is_short)
    {
      polyfit(left_line_);
      if (center_line_.index != -1 || right_line_.index != -1)
      {
        if (center_line_.index != -1 && right_line_.index != -1)
        {
          // "l+  c/  r/"
          l_state = '+';
          c_state = '/';
          r_state = '/';
          adjust(left_line_, center_line_, false);
          adjust(left_line_, right_line_, false);
        }
        else if (center_line_.index != -1)
        {
          // "l+  c/  r-"
          l_state = '+';
          c_state = '/';
          r_state = '-';
          adjust(left_line_, center_line_, false);
          right_line_.degree = center_line_.degree;
          polyfit(createOffsetLine(center_line_, -1 * right_lane_width_), right_line_);
        }
        else
        {
          // "l+  c-  r/"
          l_state = '+';
          c_state = '-';
          r_state = '/';
          adjust(left_line_, right_line_, false);
          center_line_.degree = left_line_.degree;
          polyfit(createOffsetLine(left_line_, -1 * left_lane_width_), center_line_);
        }
      }
      else
      {
        // "l+  c-  r-"
        l_state = '+';
        c_state = '-';
        r_state = '-';
        center_line_.degree = left_line_.degree;
        polyfit(createOffsetLine(left_line_, -1 * left_lane_width_), center_line_);
        right_line_.degree = center_line_.degree;
        polyfit(createOffsetLine(center_line_, -1 * right_lane_width_), right_line_);
      }
    }

    else if (!center_line_.is_short)
    {
      polyfit(center_line_);
      if (left_line_.index != -1 || right_line_.index != -1)
      {
        if (left_line_.index != -1 && right_line_.index != -1)
        {
          // "l/  c+  r/"
          l_state = '/';
          c_state = '+';
          r_state = '/';
          adjust(center_line_, left_line_, true);
          adjust(center_line_, right_line_, false);
        }
        else if (left_line_.index != -1)
        {
          // "l/  c+  r-"
          l_state = '/';
          c_state = '+';
          r_state = '-';
          adjust(center_line_, left_line_, true);
          right_line_.degree = center_line_.degree;
          polyfit(createOffsetLine(center_line_, -1 * right_lane_width_), right_line_);
        }
        else
        {
          // "l-  c+  r/"
          l_state = '-';
          c_state = '+';
          r_state = '/';
          adjust(center_line_, right_line_, false);
          left_line_.degree = center_line_.degree;
          polyfit(createOffsetLine(center_line_, left_lane_width_), left_line_);
        }
      }
      else
      {
        // "l-  c+  r-"
        l_state = '-';
        c_state = '+';
        r_state = '-';
        left_line_.degree = center_line_.degree;
        polyfit(createOffsetLine(center_line_, left_lane_width_), left_line_);
        right_line_.degree = center_line_.degree;
        polyfit(createOffsetLine(center_line_, -1 * right_lane_width_), right_line_);
      }
    }
    break;

  case 0:
    // "l-  c-  r-"
    l_state = '-';
    c_state = '-';
    r_state = '-';
    break;
  }

  if (debug_mode_)
  {
    std::cout << "--------aprox state---------" << std::endl;
    std::cout << "left_index   " << left_line_.index << std::endl;
    std::cout << "center_index " << center_line_.index << std::endl;
    std::cout << "right_index  " << right_line_.index << std::endl
              << std::endl;

    std::cout << "left_length   " << left_line_.length << std::endl;
    std::cout << "center_length " << center_line_.length << std::endl;
    std::cout << "right_length  " << right_line_.length << std::endl
              << std::endl;

    std::cout << "left_degree   " << left_line_.degree << std::endl;
    std::cout << "center_degree " << center_line_.degree << std::endl;
    std::cout << "right_degree  " << right_line_.degree << std::endl
              << std::endl;

    std::cout << "l" << l_state << "  c" << c_state << "  r" << r_state << std::endl;
  }
}

void LaneDetector::lanesVectorVisualization(cv::Mat &visualization_frame)
{
  lines_vector_.clear();
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    std::vector<cv::Point2f> line2f;
    std::vector<cv::Point> line;
    cv::transform(lines_vector_converted_[i], line2f, world2topview_.rowRange(0, 2));
    cv::Mat(line2f).copyTo(line);
    lines_vector_.push_back(line);
  }

  visualization_frame = cv::Mat::zeros(visualization_frame.size(), CV_8UC3);
  if (lines_vector_.empty())
    return;
  for (int i = 0; i < lines_vector_.size(); ++i)
  {
    for (int j = 0; j < lines_vector_[i].size(); ++j)
    {
      cv::circle(visualization_frame, lines_vector_[i][j], 2, cv::Scalar(255, 0, 0), CV_FILLED, cv::LINE_AA);
    }
  }

  int red_value = 0;
  int green_value = 0;
  for (int i = 0; i < lines_vector_.size(); ++i)
  {
    if (i % 4 == 0)
      red_value = 0;
    else if (i % 4 == 1)
      green_value = 255;
    else if (i % 4 == 2)
        red_value = 255;
    else
    {
        green_value = 125;
        red_value = 125;
    }
    
    for (int j = 1; j < lines_vector_[i].size(); ++j)
    {
      cv::line(visualization_frame, lines_vector_[i][j], lines_vector_[i][j - 1], cv::Scalar(red_value, green_value, 255), 1);
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
  if (left_line_.index > -1)
  {
    if (lines_vector_converted_[left_line_.index][0].x > ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 4))
    {
      temp.x = TOPVIEW_MIN_X;
      temp.y = getAproxY(left_line_.coeff, TOPVIEW_MIN_X);
      lines_vector_converted_[left_line_.index].insert(lines_vector_converted_[left_line_.index].begin(), temp);
    }
  }

  if (right_line_.index > -1)
  {
    if (lines_vector_converted_[right_line_.index][0].x > ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 4))
    {
      temp.x = TOPVIEW_MIN_X;
      temp.y = getAproxY(right_line_.coeff, TOPVIEW_MIN_X);
      lines_vector_converted_[right_line_.index].insert(lines_vector_converted_[right_line_.index].begin(), temp);
    }
  }

  if (center_line_.index > -1)
  {
    if (lines_vector_converted_[center_line_.index][0].x > ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 4))
    {
      temp.x = TOPVIEW_MIN_X;
      temp.y = getAproxY(center_line_.coeff, TOPVIEW_MIN_X);
      lines_vector_converted_[center_line_.index].insert(lines_vector_converted_[center_line_.index].begin(), temp);
    }
  }
}

bool LaneDetector::polyfit(std::vector<cv::Point2f> line, RoadLine &road_line)
{
  int nDegree = road_line.degree;
  if (line.size() < nDegree)
    return false;

  // more intuative this way
  ++nDegree;

  size_t nCount = line.size();
  boost::numeric::ublas::matrix<float> oXMatrix(nCount, nDegree);
  boost::numeric::ublas::matrix<float> oYMatrix(nCount, 1);

  // copy y matrix
  for (size_t i = 0; i < nCount; ++i)
  {
    oYMatrix(i, 0) = line[i].y;
  }

  // create the X matrix
  for (size_t nRow = 0; nRow < nCount; ++nRow)
  {
    float nVal = 1.0f;
    for (int nCol = 0; nCol < nDegree; ++nCol)
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
  road_line.coeff = vec;
  return true;
}

bool LaneDetector::polyfit(RoadLine &road_line)
{
  int nDegree = road_line.degree;
  if (lines_vector_converted_[road_line.index].size() < nDegree)
    return false;

  // more intuative this way
  ++nDegree;

  size_t nCount = lines_vector_converted_[road_line.index].size();
  boost::numeric::ublas::matrix<float> oXMatrix(nCount, nDegree);
  boost::numeric::ublas::matrix<float> oYMatrix(nCount, 1);

  // copy y matrix
  for (size_t i = 0; i < nCount; ++i)
  {
    oYMatrix(i, 0) = lines_vector_converted_[road_line.index][i].y;
  }

  // create the X matrix
  for (size_t nRow = 0; nRow < nCount; ++nRow)
  {
    float nVal = 1.0f;
    for (int nCol = 0; nCol < nDegree; ++nCol)
    {
      oXMatrix(nRow, nCol) = nVal;
      nVal *= lines_vector_converted_[road_line.index][nRow].x;
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
  road_line.coeff = vec;
  return true;
}

void LaneDetector::adjust(RoadLine &good_road_line,
                          RoadLine &short_road_line, bool left_offset)
{
  short_road_line.degree = good_road_line.degree;

  float step = 0.03;
  float offset = 0.2;
  int count = 0;
  if (!left_offset)
  {
    step *= -1;
    offset *= -1;
  }
  float last_cost = 9999999;  // init huge value
  while (true || std::fabs(offset) > 1)
  {
    if (polyfit(createOffsetLine(good_road_line, offset), short_road_line))
    {
      float current_cost = 0;
      for (int i = 0; i < lines_vector_converted_[short_road_line.index].size() - 1; ++i)
      {
        float aprox = getAproxY(short_road_line.coeff, lines_vector_converted_[short_road_line.index][i].x);
        current_cost += std::fabs(aprox - lines_vector_converted_[short_road_line.index][i].y);
      }
      if (current_cost > last_cost)
        break;
      else
        last_cost = current_cost;
    }
    offset += step;
    ++count;
  }
  offset -= step;
  if (!polyfit(createOffsetLine(good_road_line, offset), short_road_line))
  {
    std::vector<float> coeff = good_road_line.coeff;
    if (left_offset)
      good_road_line.coeff[0] += left_lane_width_;
    else
      good_road_line.coeff[0] -= right_lane_width_;
    short_road_line.coeff = coeff;
  }
}

void LaneDetector::calcRoadWidth()
{
  cv::Point2f p;
  cv::Point2f p_ahead;
  p_ahead.x = 0.6;
  p_ahead.y = getAproxY(center_line_.coeff, p_ahead.x);
  double deriative;
  if (center_line_.degree == 3)
    deriative = 3 * center_line_.coeff[3] * pow(p_ahead.x, 2)
              + 2 * center_line_.coeff[2] * p_ahead.x
              + center_line_.coeff[1];
  else
    deriative = 2 * center_line_.coeff[2] * p_ahead.x
              + center_line_.coeff[1];

  double a_param_orthg = -1 / deriative;
  double b_param_orthg = p_ahead.y - a_param_orthg * p_ahead.x;

  // right lane
  if (right_line_.index != -1)
  {
    p.y = p_ahead.y - 0.3;
    float step = -0.01;
    float last_dist = 9999999;  // init huge value

    while (true || std::fabs(p.y) > 1)
    {
      p.x = (p.y - b_param_orthg) / a_param_orthg;
      cv::Point2f p_aprox;
      p_aprox.x = p.x;
      p_aprox.y = getAproxY(right_line_.coeff, p_aprox.x);
      float dist = getDistance(p, p_aprox);
      if (dist - last_dist > 0)
      {
        break;
      }
      else
      {
        last_dist = dist;
        p.y += step;
      }
    }
    float lane_width = getDistance(p_ahead, p);
    if (lane_width > 0.3 && lane_width < 0.5)
      right_lane_width_ = lane_width;
  }

  // left lane
  if (left_line_.index != -1)
  {
    p.y = p_ahead.y + 0.3;
    float step = 0.01;
    float last_dist = 9999999;  // init huge value

    while (true || std::fabs(p.y) > 1)
    {
      p.x = (p.y - b_param_orthg) / a_param_orthg;
      cv::Point2f p_aprox;
      p_aprox.x = p.x;
      p_aprox.y = getAproxY(left_line_.coeff, p_aprox.x);
      float dist = getDistance(p, p_aprox);
      if (dist - last_dist > 0)
      {
        break;
      }
      else
      {
        last_dist = dist;
        p.y += step;
      }
    }
    float lane_width = getDistance(p_ahead, p);
    if (lane_width > 0.3 && lane_width < 0.5)
      left_lane_width_ = lane_width;
  }
}

void LaneDetector::generatePoints()
{
  if (left_line_.index != -1)
  {
    if (lines_vector_converted_[left_line_.index].size()
        / cv::arcLength(lines_vector_converted_[left_line_.index], false) < points_density_ * 2)
    {
      for (int i = 0; i < lines_vector_converted_[left_line_.index].size() - 1; ++i)
      {
        float distance = getDistance(lines_vector_converted_[left_line_.index][i],
                                     lines_vector_converted_[left_line_.index][i + 1]);
        if (distance > 1 / points_density_)
        {
          int add = distance * points_density_;
          cv::Point2f p;
          float x1 = lines_vector_converted_[left_line_.index][i].x;
          float y1 = lines_vector_converted_[left_line_.index][i].y;
          float x_dif = (lines_vector_converted_[left_line_.index][i + 1].x
                        - lines_vector_converted_[left_line_.index][i].x) / (add + 1);
          float y_dif = (lines_vector_converted_[left_line_.index][i + 1].y
                        - lines_vector_converted_[left_line_.index][i].y) / (add + 1);
          for (int j = 0; j < add; ++j)
          {
            p.x = x1 + x_dif * (j + 1);
            p.y = y1 + y_dif * (j + 1);
            lines_vector_converted_[left_line_.index].insert(lines_vector_converted_[left_line_.index].begin()
                                                            + i + 1, p);
            ++i;
          }
        }
      }
    }
  }

  if (center_line_.index != -1)
  {
    if (lines_vector_converted_[center_line_.index].size()
        / cv::arcLength(lines_vector_converted_[center_line_.index], false) < points_density_ * 2)
    {
      for (int i = 0; i < lines_vector_converted_[center_line_.index].size() - 1; ++i)
      {
        float distance = getDistance(lines_vector_converted_[center_line_.index][i],
                                     lines_vector_converted_[center_line_.index][i + 1]);
        if (distance > 1 / points_density_)
        {
          int add = distance * points_density_;
          cv::Point2f p;
          float x1 = lines_vector_converted_[center_line_.index][i].x;
          float y1 = lines_vector_converted_[center_line_.index][i].y;
          float x_dif = (lines_vector_converted_[center_line_.index][i + 1].x
                        - lines_vector_converted_[center_line_.index][i].x) / (add + 1);
          float y_dif = (lines_vector_converted_[center_line_.index][i + 1].y
                        - lines_vector_converted_[center_line_.index][i].y) / (add + 1);
          for (int j = 0; j < add; ++j)
          {
            p.x = x1 + x_dif * (j + 1);
            p.y = y1 + y_dif * (j + 1);
            lines_vector_converted_[center_line_.index].insert(lines_vector_converted_[center_line_.index].begin()
                                                              + i + 1, p);
            ++i;
          }
        }
      }
    }
  }

  if (right_line_.index != -1)
  {
    if (lines_vector_converted_[right_line_.index].size()
        / cv::arcLength(lines_vector_converted_[right_line_.index], false) < points_density_ * 2)
    {
      for (int i = 0; i < lines_vector_converted_[right_line_.index].size() - 1; ++i)
      {
        float distance = getDistance(lines_vector_converted_[right_line_.index][i],
                                     lines_vector_converted_[right_line_.index][i + 1]);
        if (distance > 1 / points_density_)
        {
          int add = distance * points_density_;
          cv::Point2f p;
          float x1 = lines_vector_converted_[right_line_.index][i].x;
          float y1 = lines_vector_converted_[right_line_.index][i].y;
          float x_dif = (lines_vector_converted_[right_line_.index][i + 1].x
                        - lines_vector_converted_[right_line_.index][i].x) / (add + 1);
          float y_dif = (lines_vector_converted_[right_line_.index][i + 1].y
                        - lines_vector_converted_[right_line_.index][i].y) / (add + 1);
          for (int j = 0; j < add; ++j)
          {
            p.x = x1 + x_dif * (j + 1);
            p.y = y1 + y_dif * (j + 1);
            lines_vector_converted_[right_line_.index].insert(lines_vector_converted_[right_line_.index].begin()
                                                              + i + 1, p);
            ++i;
          }
        }
      }
    }
  }
}

void LaneDetector::removeHorizontalLines()
{
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    float dx = lines_vector_converted_[i][lines_vector_converted_[i].size() - 1].x - lines_vector_converted_[i][0].x;
    if (dx < 0.1)
      lines_vector_converted_.erase(lines_vector_converted_.begin() + i);
  }
}

std::vector<cv::Point2f> LaneDetector::createOffsetLine(RoadLine &road_line, float offset)
{
  std::vector<cv::Point2f> new_line;
  cv::Point2f p;
  for (float i = TOPVIEW_MIN_X - 0.2; i < TOPVIEW_MAX_X + 0.2; i += 0.05)
  {
    float deriative;
    switch (road_line.degree)
    {
    case 2:
      deriative = 2 * road_line.coeff[2] * i + road_line.coeff[1];
      break;
    case 3:
      deriative = 3 * road_line.coeff[3] * pow(i, 2) + 2 * road_line.coeff[2] * i + road_line.coeff[1];
      break;
    case 1:
      deriative = road_line.coeff[1];
      break;
    }

    float angle = atan(deriative);
    p.x = i - offset * sin(angle);
    p.y = getAproxY(road_line.coeff, i) + offset * cos(angle);
    new_line.push_back(p);
  }
  return new_line;
}

void LaneDetector::detectStartAndIntersectionLine()
{
  starting_line_ = false;
  float detect_slope = 1.0;
  float detect_lenght = 0.15;
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
  for (int i = 0; i < right_lines.size(); ++i)
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
  for (int i = 0; i < left_lines.size(); ++i)
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

  if(debug_mode_)
	{
		cv::Mat rlf, llf;

		rlf = cv::Mat::zeros(homography_frame_.size(), CV_8UC3);
		llf = cv::Mat::zeros(homography_frame_.size(), CV_8UC3);

		cv::drawContours(llf, left_lines, -1, cv::Scalar(255,0,0), 2);
		cv::drawContours(rlf, right_lines, -1, cv::Scalar(255,0,0), 2);

		for(int i = 0; i < left_lines.size(); i++)
		{
			cv::RotatedRect rect = cv::minAreaRect(left_lines[i]);
			cv::Point2f vertices2f[4];
    		rect.points(vertices2f);

    		// Convert them so we can use them in a fillConvexPoly
    		cv::Point vertices[4];    
    		for(int i = 0; i < 4; ++i){
        		vertices[i] = vertices2f[i];
    		}
			cv::fillConvexPoly(llf, vertices,4, cv::Scalar(0,0,255));
		}
		for(int i = 0; i < right_lines.size(); i++)
		{
			cv::RotatedRect rect = cv::minAreaRect(right_lines[i]);

			cv::Point2f vertices2f[4];
    		rect.points(vertices2f);
			//ROS_INFO("1: %.4f       2: %.4f    3: %.4f   4: %.4f", vertices2f[0].x, vertices2f[1].x, vertices2f[2].x, vertices2f[3].x);

    		// Convert them so we can use them in a fillConvexPoly
    		cv::Point vertices[4];    
    		for(int i = 0; i < 4; ++i){
        		vertices[i] = vertices2f[i];
			}
			cv::fillConvexPoly(rlf, vertices,4, cv::Scalar(0,0,255));
		}

    cv::namedWindow("LANE_DEBUG", cv::WINDOW_NORMAL);
    cv::Mat m_raw, m_filtered, m_all;
    cv::hconcat(left_lane_frame_, right_lane_frame_, m_raw);
    cv::cvtColor(m_raw, m_raw, CV_GRAY2BGR);
    cv::hconcat(llf, rlf, m_filtered);
    cv::vconcat(m_raw, m_filtered, m_all);
    cv::imshow("LANE_DEBUG", m_all);
	}

  if (std::abs(right_distance - left_distance) < 0.25 && left_distance > 0)
  {
    std_msgs::Float32 msg;
    msg.data = right_distance;
    starting_line_pub_.publish(msg);
    starting_line_ = true;
  }
  else if (right_distance > 0)
  {
    std_msgs::Float32 msg;
    msg.data = right_distance;
    intersection_pub_.publish(msg);
  }
}

void LaneDetector::calcRoadLinesParams()
{
  if (left_line_.index != -1)
  {
    left_line_.length = cv::arcLength(lines_vector_converted_[left_line_.index], false);
    if (left_line_.length > min_length_to_2aprox_)
      left_line_.is_short = false;
    else
      left_line_.is_short = true;
  }
  else
  {
    left_line_.length = 0;
    left_line_.is_short = true;
  }

  if (right_line_.index != -1)
  {
    right_line_.length = cv::arcLength(lines_vector_converted_[right_line_.index], false);
    if (right_line_.length > min_length_to_2aprox_)
      right_line_.is_short = false;
    else
      right_line_.is_short = true;
  }
  else
  {
    right_line_.length = 0;
    right_line_.is_short = true;
  }

  if (center_line_.index != -1)
  {
    center_line_.length = cv::arcLength(lines_vector_converted_[center_line_.index], false);
    if (center_line_.length > min_length_to_2aprox_)
      center_line_.is_short = false;
    else
      center_line_.is_short = true;
  }
  else
  {
    center_line_.length = 0;
    center_line_.is_short = true;
  }
}
