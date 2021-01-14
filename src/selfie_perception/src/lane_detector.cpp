/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

// TODO fun aprox - przesuwanie tym samym stopniem

#include <selfie_perception/lane_detector.h>

static int Acc_value = 1;

LaneDetector::LaneDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) :
  nh_(nh),
  pnh_(pnh),
  it_(nh)
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
  lines_vector_.clear();
  aprox_lines_frame_coordinate_.clear();
  std::vector<cv::Point2f> empty;
  empty.clear();
  aprox_lines_frame_coordinate_.push_back(empty);
  aprox_lines_frame_coordinate_.push_back(empty);
  aprox_lines_frame_coordinate_.push_back(empty);

  getParams();

  kernel_v_ = cv::Mat(1, 3, CV_32F);
  kernel_v_.at<float>(0, 0) = -1;
  kernel_v_.at<float>(0, 1) = 0;
  kernel_v_.at<float>(0, 2) = 1;

  kernel_h_ = cv::Mat(3, 1, CV_32F);
  kernel_h_.at<float>(0, 0) = -1;
  kernel_h_.at<float>(1, 0) = 0;
  kernel_h_.at<float>(2, 0) = 1;
  dilate_element_ = getStructuringElement(2, cv::Size(3, 3), cv::Point(1, 1));
  close_element_ = getStructuringElement(0, cv::Size(7, 7), cv::Point(3, 3));
  dilate_obst_element_ = getStructuringElement(0, cv::Size(15, 15), cv::Point(7, 7));

  int obs_el_size = static_cast<int>(TOPVIEW_COLS / (TOPVIEW_MAX_Y - TOPVIEW_MIN_Y) * obstacle_window_size_);
  if (obs_el_size % 2 == 0)
    obs_el_size++;
  obstacle_element_ = getStructuringElement(2, cv::Size(obs_el_size, obs_el_size), cv::Point(obs_el_size / 2, obs_el_size / 2));

  treshold_block_size_ = static_cast<int>(TOPVIEW_COLS / (TOPVIEW_MAX_Y - TOPVIEW_MIN_Y) * real_window_size_);
  if (treshold_block_size_ % 2 == 0)
    treshold_block_size_++;

  image_sub_ = it_.subscribe("/image_rect", 10, &LaneDetector::imageCallback, this);
  if (debug_mode_)
  {
    points_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("new_coordinates", 10);
  }

  left_line_.setShortParam(min_length_to_2aprox_);
  center_line_.setShortParam(min_length_to_2aprox_);
  right_line_.setShortParam(min_length_to_2aprox_);

  left_line_.setPointsDensity(points_density_);
  center_line_.setPointsDensity(points_density_);
  right_line_.setPointsDensity(points_density_);

  left_line_.pfSetup(pf_num_samples_, pf_num_points_, pf_std_min_, pf_std_max_);
  center_line_.pfSetup(pf_num_samples_, pf_num_points_, pf_std_min_, pf_std_max_);
  right_line_.pfSetup(pf_num_samples_, pf_num_points_, pf_std_min_, pf_std_max_);

  computeTopView();
  printInfoParams();
  if (tune_params_mode_)
  {
    current_frame_ = cv::Mat::zeros(cv::Size(TOPVIEW_COLS, TOPVIEW_ROWS), CV_8UC1);
    tune_timer_ = nh_.createTimer(ros::Duration(0.01), &LaneDetector::tuneParams, this);
  }
  outside_road_ = cv::Mat::zeros(cv::Size(TOPVIEW_COLS, TOPVIEW_ROWS), CV_8UC1);
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
  //cv::GaussianBlur(homography_frame_, homography_frame_, cv::Size(5,5), 0);
  createObstaclesMask();

  cv::bitwise_or(hom_cut_mask_, homography_frame_, homography_frame_);

  cv::adaptiveThreshold(homography_frame_, binary_frame_, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                        CV_THRESH_BINARY, treshold_block_size_, threshold_c_);

  cv::bitwise_and(binary_frame_, obstacles_mask_, binary_frame_);
  cv::bitwise_and(binary_frame_, hom_cut_mask_inv_, binary_frame_);

  cv::medianBlur(binary_frame_, binary_frame_, 3);
  removeCar(binary_frame_);

  if (!init_imageCallback_)
  {
    dynamicMask(binary_frame_, masked_frame_);
      
    // remove ROI inside left and right lane
    ROILaneRight(masked_frame_, masked_frame_);
    ROILaneLeft(masked_frame_, masked_frame_);
    if (debug_mode_)
    {
      cv::Mat copy = homography_frame_.clone();
      cv::bitwise_and(homography_frame_, dynamic_mask_, copy);
      //bitwise_not(dynamic_mask_, dynamic_mask_);
      cv::addWeighted(copy, 0.7, homography_frame_, 0.3, 0, homography_masked_frame_);

      copy = homography_masked_frame_.clone();
      cv::bitwise_and(homography_masked_frame_, left_lane_ROI_, copy);
      cv::addWeighted(copy, 0.7, homography_masked_frame_, 0.3, 0, homography_masked_frame_);

      copy = homography_masked_frame_.clone();
      cv::bitwise_and(homography_masked_frame_, right_lane_ROI_, copy);
      cv::addWeighted(copy, 0.7, homography_masked_frame_, 0.3, 0, homography_masked_frame_);
    }
    detectStartAndIntersectionLine();

    cv::bitwise_not(dynamic_mask_, dynamic_mask_);
    cv::bitwise_and(binary_frame_, dynamic_mask_, outside_road_);
    cv::filter2D(outside_road_, outside_road_, -1, kernel_h_, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    cv::HoughLinesP(outside_road_, lines_out_h_, isec_HL_dist_res_, isec_HL_angle_res_,
                    isec_HL_thresh_, isec_HL_min_length_, isec_HL_max_gap_);
  }
  else
  {
    masked_frame_=  binary_frame_.clone();
    if (debug_mode_)
      homography_masked_frame_ = homography_frame_.clone();
  }
    

  cv::filter2D(masked_frame_, masked_frame_, -1, kernel_v_, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
  dilate(masked_frame_, masked_frame_, dilate_element_);

  detectLines(masked_frame_, lines_vector_);
  if (lines_vector_.empty())
  {
    left_line_.setExist(false);
    center_line_.setExist(false);
    right_line_.setExist(false);
    publishMarkings();
    return;
  }
  convertCoordinates();
  filterSmallLines();
  if (lines_vector_converted_.empty())
  {
    left_line_.setExist(false);
    center_line_.setExist(false);
    right_line_.setExist(false);
    publishMarkings();
    return;
  }

  mergeMiddleLines();

  if (init_imageCallback_)
  {
    initRecognizeLines();
    right_line_.calcParams();
    center_line_.calcParams();
    left_line_.calcParams();
    int l = 0, c = 0, r = 0;
    if (!left_line_.isShort())
      l = 1;

    if (!center_line_.isShort())
      c = 1;

    if (!right_line_.isShort())
      r = 1;

    if ((l + r + c) > 1)
    {
      linesApproximation();
      init_imageCallback_ = false;
    }
  }
  else
  {
    recognizeLinesNew();

    if(!isIntersection())
    {
      if (waiting_for_stabilize_)
      {
        float half_of_image = (TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 2.0;
        if (right_line_.isExist() && right_line_.getPoints()[0].x < half_of_image && right_line_.getPoints()[right_line_.pointsSize()].x > half_of_image
         || left_line_.isExist() && left_line_.getPoints()[0].x < half_of_image && left_line_.getPoints()[left_line_.pointsSize()].x > half_of_image
         || center_line_.isExist() && center_line_.getPoints()[0].x < half_of_image && center_line_.getPoints()[center_line_.pointsSize()].x > half_of_image)
        {
          if (right_line_.getMaxDiffonX() < 0.3 && left_line_.getMaxDiffonX() < 0.3 && center_line_.getMaxDiffonX() < 0.3)
          {
            waiting_for_stabilize_ = false;
            ROS_INFO("waiting_for_stabilize is false");
            center_line_.setDegree(2);
            right_line_.setDegree(2);
            left_line_.setDegree(2);
            center_line_.pfReset();
            right_line_.pfReset();
            left_line_.pfReset();
          }
        }
      }
      right_line_.addBottomPoint(waiting_for_stabilize_);
      center_line_.addBottomPoint(waiting_for_stabilize_);
      left_line_.addBottomPoint(waiting_for_stabilize_);

      right_line_.calcParams();
      center_line_.calcParams();
      left_line_.calcParams();

      if (!center_line_.isShort())
        calcRoadWidth();

      right_line_.generateForDensity();
      center_line_.generateForDensity();
      left_line_.generateForDensity();
      linesApproximation();
    }
      
  }
  publishMarkings();

  if (debug_mode_)
  {
    debug_frame_.rows = homography_frame_.rows;
    debug_frame_.cols = homography_frame_.cols;
    lanesVectorVisualization(debug_frame_);
    
    drawParticles(pf_num_samples_vis_);

    convertApproxToFrameCoordinate();
    drawAproxOnHomographyMasked();
    openCVVisualization();
    pointsRVIZVisualization();
  }
}

bool LaneDetector::resetVisionCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  init_imageCallback_ = true;
  left_line_.reset();
  center_line_.reset();
  right_line_.reset();
  intersection_line_dist_ = -1;
  proof_intersection_ = 0;
  proof_start_line_ = 0;
  intersection_ = false;
  waiting_for_stabilize_ = false;
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

  for (int i = 0; i < output_lanes.size(); ++i)
  {
    cv::approxPolyDP(cv::Mat(output_lanes[i]), output_lanes[i], Acc_value, false);
  }
}

void LaneDetector::drawAproxOnHomographyMasked()
{
  if(homography_masked_frame_.type() == 0)
    cv::cvtColor(homography_masked_frame_, homography_masked_frame_, CV_GRAY2BGR);
  if (!aprox_lines_frame_coordinate_[2].empty())
    for (int i = 0; i < aprox_lines_frame_coordinate_[2].size(); ++i)
    {
      cv::circle(homography_masked_frame_, aprox_lines_frame_coordinate_[2][i], 2, cv::Scalar(255, 0, 0), cv::FILLED);
    }
  if (!aprox_lines_frame_coordinate_[0].empty())
    for (int i = 0; i < aprox_lines_frame_coordinate_[0].size(); ++i)
    {
      cv::circle(homography_masked_frame_, aprox_lines_frame_coordinate_[0][i], 2, cv::Scalar(0, 0, 255), cv::FILLED);
    }
  if (!aprox_lines_frame_coordinate_[1].empty())
    for (int i = 0; i < aprox_lines_frame_coordinate_[1].size(); ++i)
    {
      cv::circle(homography_masked_frame_, aprox_lines_frame_coordinate_[1][i], 2, cv::Scalar(0, 255, 0), cv::FILLED);
    }

  if (!debug_points_.empty())
  {
    cv::transform(debug_points_, debug_points_, world2topview_.rowRange(0, 2));
    for (int i = 0; i < debug_points_.size(); i += 2)
    {
      cv::line(homography_masked_frame_, debug_points_[i], debug_points_[i + 1], cv::Scalar(0, 255, 255), 2);
	  cv::circle(homography_masked_frame_, debug_points_[i], 5, cv::Scalar(255, 255, 0), cv::FILLED, cv::LINE_AA);
	  cv::circle(homography_masked_frame_, debug_points_[i + 1], 5, cv::Scalar(255, 255, 0), cv::FILLED, cv::LINE_AA);
    }
  }
}

void LaneDetector::homography(cv::Mat input_frame, cv::Mat &homography_frame)
{
  cv::warpPerspective(input_frame, homography_frame, topview2cam_,
                      topview_size_, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
}

void LaneDetector::getParams()
{
  cv::FileStorage fs1;
  if (pnh_.getParam("config_file", config_file_) && fs1.open(config_file_, cv::FileStorage::READ))
  {
    fs1["world2cam"] >> world2cam_;
    fs1.release();
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

  cv::FileStorage fs2;
  if (pnh_.getParam("hom_cut_file", hom_cut_file_) && fs2.open(hom_cut_file_, cv::FileStorage::READ))
  {
    fs2["mat"] >> hom_cut_mask_;
    fs2.release();
  }
  else
  {
    // zero hom_cut mask
    ROS_INFO("Zero hom_cut mask");
    hom_cut_mask_ = cv::Mat::zeros(cv::Size(TOPVIEW_COLS, TOPVIEW_ROWS), CV_8UC1);
  }
  cv::bitwise_not(hom_cut_mask_, hom_cut_mask_inv_);
  
  pnh_.getParam("real_window_size", real_window_size_);
  pnh_.getParam("threshold_c", threshold_c_);
  pnh_.getParam("debug_mode", debug_mode_);
  pnh_.getParam("tune_params_mode", tune_params_mode_);
  pnh_.getParam("max_mid_line_distance", max_mid_line_distance_);
  pnh_.getParam("max_mid_line_gap", max_mid_line_gap_);

  pnh_.getParam("pf_num_samples", pf_num_samples_);
  pnh_.getParam("pf_num_points_", pf_num_points_);
  pnh_.getParam("pf_std_min", pf_std_min_);
  pnh_.getParam("pf_std_max", pf_std_max_);
  pnh_.getParam("pf_num_samples_vis", pf_num_samples_vis_);

  pnh_.getParam("obstacle_window_size", obstacle_window_size_);
  pnh_.getParam("obstacles_threshold", obstacles_threshold_);
}

void LaneDetector::openCVVisualization()
{
  cv::namedWindow("STD_DEBUG", cv::WINDOW_NORMAL);
  cv::Mat m_binary, m_std, m_four;
  cv::hconcat(binary_frame_, masked_frame_, m_binary);
  cv::cvtColor(m_binary, m_binary, CV_GRAY2BGR);
  cv::hconcat(debug_frame_, homography_masked_frame_, m_std);
  cv::vconcat(m_binary, m_std, m_four);

  cv::imshow("STD_DEBUG", m_four);

  cv::namedWindow("ADV_DEBUG", cv::WINDOW_NORMAL);
  cv::Mat m_pf, m_obs_inter, m_all;

  cv::Mat LCRLines;
  LCRLines = cv::Mat::zeros(homography_frame_.size(), CV_8UC3);
  LCRLinesDraw(LCRLines);

  cv::hconcat(LCRLines, pf_vis_mat_, m_pf);
  cv::bitwise_not(obstacles_mask_, obstacles_mask_);
  cv::cvtColor(obstacles_mask_, obstacles_mask_, CV_GRAY2BGR);
  drawIntersection();
  cv::hconcat(obstacles_mask_, outside_road_, m_obs_inter);
  cv::vconcat(m_pf, m_obs_inter, m_all);
  cv::imshow("ADV_DEBUG", m_all);

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
    if (!init_imageCallback_)
    {
        if (lines_vector_converted_[i][0].y < getPolyY(right_line_.getCoeff(), lines_vector_converted_[i][0].x) + 0.05 ||
            lines_vector_converted_[i][0].y > getPolyY(left_line_.getCoeff(), lines_vector_converted_[i][0].x) - 0.05)
            {
              continue;
            }
    }
    float a, b;
    for (int j = i + 1; j < lines_vector_converted_.size(); ++j)
    {
      if (!init_imageCallback_)
      {
        if (lines_vector_converted_[j][0].y < getPolyY(right_line_.getCoeff(), lines_vector_converted_[j][0].x) + 0.05 ||
            lines_vector_converted_[j][0].y > getPolyY(left_line_.getCoeff(), lines_vector_converted_[j][0].x) - 0.05) 
            {
              continue;
            }
          
      }
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

void LaneDetector::recognizeLinesNew()
{
  float checking_length = 0.4;
  float max_cost = 0.05;
  if (intersection_ || waiting_for_stabilize_)
    max_cost = 0.1;
  float ratio = 0.4;
  center_line_.clearPoints();
  right_line_.clearPoints();
  left_line_.clearPoints();
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    float center_cost = 0;
    float right_cost = 0;
    float left_cost = 0;
    float start_checking_x = lines_vector_converted_[i][0].x;
    int count = 0;
    for (int j = 0; j < lines_vector_converted_[i].size(); ++j)
    {
      float weight = 1 - (lines_vector_converted_[i][j].x - TOPVIEW_MIN_X) * ratio;
      //std::cout << "weight: " << weight << std::endl;
      //std::cout << "x: " << lines_vector_converted_[i][j].x << std::endl;
      center_cost += weight * findMinPointToParabola(lines_vector_converted_[i][j], center_line_.getCoeff());
      right_cost += weight * findMinPointToParabola(lines_vector_converted_[i][j], right_line_.getCoeff());
      left_cost += weight * findMinPointToParabola(lines_vector_converted_[i][j], left_line_.getCoeff());
      ++count;
      if (lines_vector_converted_[i][j].x - start_checking_x > checking_length)
      {
        break;
      }
    }
    center_cost /= count;
    right_cost /= count;
    left_cost /= count;

    center_cost = std::fabs(center_cost);
    right_cost = std::fabs(right_cost);
    left_cost = std::fabs(left_cost);

    float min = std::min(std::min(center_cost, right_cost), left_cost);

    if (min == center_cost && center_cost < max_cost)
    {
      center_line_.addPoints(lines_vector_converted_[i]);
    }
    else if (min == right_cost && right_cost < max_cost)
    {
      right_line_.addPoints(lines_vector_converted_[i]);
    }
    else if (min == left_cost && left_cost < max_cost)
    {
      left_line_.addPoints(lines_vector_converted_[i]);
    }

    //std::cout << "center_cost: " << center_cost << std::endl;
    //std::cout << "right_cost: " << right_cost << std::endl;
    //std::cout << "left_cost: " << left_cost << std::endl;
    //std::cout << "--------------------------" << std::endl;
  }
}

float LaneDetector::findMinPointToParabola(cv::Point2f p, std::vector<float> coeff)
{
  cv::Point2f poly_p;
  poly_p.x = p.x;
  poly_p.y = getPolyY(coeff, p.x);
  float min = std::abs(p.y - poly_p.y);
  float new_min = min;
  float step = 0.05;
  int it = 0;
  do
  {
    min = new_min;
    poly_p.x -= step;
    poly_p.y = getPolyY(coeff, poly_p.x);
    new_min = getDistance(p, poly_p);
    ++it;
    if (it > 5)
      break;
  } while (new_min - min < 0);
  return min;
}

void LaneDetector::recognizeLines()
{
  center_line_.clearPoints();
  right_line_.clearPoints();
  left_line_.clearPoints();
  int left_index = -1;
  int right_index = -1;
  int center_index = -1;
  std::vector<std::vector<cv::Point2f>> closest_container;
  std::vector<int> closest_index;
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    float aprox_y = getPolyY(left_line_.getCoeff(), lines_vector_converted_[i][0].x);
    if (std::abs(aprox_y - lines_vector_converted_[i][0].y) < max_delta_y_lane_)
    {
      closest_container.push_back(lines_vector_converted_[i]);
      closest_index.push_back(i);
    }
  }
  if (closest_container.empty())
    left_line_.setExist(false);
  else
  {
    float aprox_y = getPolyY(left_line_.getCoeff(), closest_container[0][0].x);
    float min_sum_distance = std::abs(aprox_y - closest_container[0][0].y);
    aprox_y = getPolyY(left_line_.getCoeff(), closest_container[0][closest_container[0].size() / 2].x);
    min_sum_distance += std::abs(aprox_y - closest_container[0][closest_container[0].size() / 2].y);
    left_index = closest_index[0];

    for (int i = 1; i < closest_container.size(); ++i)
    {
      aprox_y = getPolyY(left_line_.getCoeff(), closest_container[i][0].x);
      float sum_distance = std::abs(aprox_y - closest_container[i][0].y);
      aprox_y = getPolyY(left_line_.getCoeff(), closest_container[i][closest_container[i].size() / 2].x);
      sum_distance += std::abs(aprox_y - closest_container[i][closest_container[i].size() / 2].y);
      if (sum_distance < min_sum_distance)
      {
        left_index = closest_index[i];
        min_sum_distance = sum_distance;
      }
    }
    left_line_.addPoints(lines_vector_converted_[left_index]);
    left_line_.setExist(true);
  }
  closest_container.clear();
  closest_index.clear();

  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    if (i == left_index)
      continue;
    float aprox_y = getPolyY(right_line_.getCoeff(), lines_vector_converted_[i][0].x);
    if (std::abs(aprox_y - lines_vector_converted_[i][0].y) < max_delta_y_lane_)
    {
      closest_container.push_back(lines_vector_converted_[i]);
      closest_index.push_back(i);
    }
  }
  if (closest_container.empty())
    right_line_.setExist(false);
  else
  {
    float aprox_y = getPolyY(right_line_.getCoeff(), closest_container[0][0].x);
    float min_sum_distance = std::abs(aprox_y - closest_container[0][0].y);
    aprox_y = getPolyY(right_line_.getCoeff(), closest_container[0][closest_container[0].size() / 2].x);
    min_sum_distance += std::abs(aprox_y - closest_container[0][closest_container[0].size() / 2].y);
    right_index = closest_index[0];

    for (int i = 1; i < closest_container.size(); ++i)
    {
      aprox_y = getPolyY(right_line_.getCoeff(), closest_container[i][0].x);
      float sum_distance = std::abs(aprox_y - closest_container[i][0].y);
      aprox_y = getPolyY(right_line_.getCoeff(), closest_container[i][closest_container[i].size() / 2].x);
      sum_distance += std::abs(aprox_y - closest_container[i][closest_container[i].size() / 2].y);
      if (sum_distance < min_sum_distance)
      {
        right_index = closest_index[i];
        min_sum_distance = sum_distance;
      }
    }
    right_line_.addPoints(lines_vector_converted_[right_index]);
    right_line_.setExist(true);
  }
  closest_container.clear();
  closest_index.clear();

  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    if (i == left_index || i == right_index)
      continue;
    float aprox_y = getPolyY(center_line_.getCoeff(), lines_vector_converted_[i][0].x);
    if (std::abs(aprox_y - lines_vector_converted_[i][0].y) < max_delta_y_lane_)
    {
      closest_container.push_back(lines_vector_converted_[i]);
      closest_index.push_back(i);
    }
  }
  if (closest_container.empty())
    center_line_.setExist(false);
  else
  {
    float aprox_y = getPolyY(center_line_.getCoeff(), closest_container[0][0].x);
    float min_sum_distance = std::abs(aprox_y - closest_container[0][0].y);
    aprox_y = getPolyY(center_line_.getCoeff(), closest_container[0][closest_container[0].size() / 2].x);
    min_sum_distance += std::abs(aprox_y - closest_container[0][closest_container[0].size() / 2].y);
    center_index = closest_index[0];

    for (int i = 1; i < closest_container.size(); ++i)
    {
      aprox_y = getPolyY(center_line_.getCoeff(), closest_container[i][0].x);
      float sum_distance = std::abs(aprox_y - closest_container[i][0].y);
      aprox_y = getPolyY(center_line_.getCoeff(), closest_container[i][closest_container[i].size() / 2].x);
      sum_distance += std::abs(aprox_y - closest_container[i][closest_container[i].size() / 2].y);
      if (sum_distance < min_sum_distance)
      {
        center_index = closest_index[i];
        min_sum_distance = sum_distance;
      }
    }
    center_line_.addPoints(lines_vector_converted_[center_index]);
    center_line_.setExist(true);
  }
}

void LaneDetector::publishMarkings()
{
  selfie_msgs::RoadMarkings road_markings;
  road_markings.header.stamp = ros::Time::now();
  road_markings.header.frame_id = "road_markings";
  for (int i = 0; i < left_line_.getCoeff().size(); ++i)
  {
    road_markings.left_line.push_back(left_line_.getCoeff()[i]);
    road_markings.right_line.push_back(right_line_.getCoeff()[i]);
    road_markings.center_line.push_back(center_line_.getCoeff()[i]);
  }
  lanes_pub_.publish(road_markings);
}

void LaneDetector::printInfoParams()
{
  ROS_INFO("real_window_size: %.3f", real_window_size_);
  ROS_INFO("treshold_block_size: %d", treshold_block_size_);
  ROS_INFO("threshold_c: %d", threshold_c_);
  ROS_INFO("max_mid_line_gap: %.3f", max_mid_line_gap_);
  ROS_INFO("max_mid_line_distance: %.3f", max_mid_line_distance_);

  ROS_INFO("debug_mode: %d\n", debug_mode_);

  ROS_INFO("pf_num_samples: %d", pf_num_samples_);
  ROS_INFO("pf_num_points: %d\n", pf_num_points_);
  ROS_INFO("pf_std_min: %.3f", pf_std_min_);
  ROS_INFO("pf_std_max: %.3f", pf_std_max_);
}

void LaneDetector::dynamicMask(cv::Mat &input_frame, cv::Mat &output_frame)
{
  dynamic_mask_ = cv::Mat::zeros(cv::Size(input_frame.cols, input_frame.rows), CV_8UC1);
  float offset_right = -0.05;
  float offset_left = 0.03;
  output_frame = input_frame.clone();
  if (!right_line_.isExist())
    offset_right = -0.10;
  else if (right_line_.getPoints()[right_line_.getPoints().size() - 1].x < ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 4))
    offset_right = -0.10;
  if (!left_line_.isExist())
    offset_left = 0.08;
  else if (left_line_.getPoints()[left_line_.getPoints().size() - 1].x < ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 4))
    offset_left = 0.08;

  std::vector<cv::Point2f> left_line_offset = createOffsetLine(left_line_.getCoeff(), left_line_.getDegree(), offset_left);
  std::vector<cv::Point2f> right_line_offset = createOffsetLine(right_line_.getCoeff(), right_line_.getDegree(), offset_right);
  left_line_offset.push_back(cv::Point2f(TOPVIEW_MAX_X, left_line_offset[left_line_offset.size() - 1].y));
  right_line_offset.push_back(cv::Point2f(TOPVIEW_MAX_X, right_line_offset[right_line_offset.size() - 1].y));
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
  float offset_center = -0.08;
  float offset_right = 0.06;
  if (starting_line_timeout_ > 0)
  {
    offset_right = 0.02;
    offset_center = -0.03;
  }
  else if (!intersection_)
  {
    if (!right_line_.isExist() || right_line_.isShort())
      offset_right = 0.08;
    if (!center_line_.isExist() || center_line_.isShort())
      offset_center = -0.1;
  }

  std::vector<cv::Point2f> center_line_offset = createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), offset_center);
  std::vector<cv::Point2f> right_line_offset = createOffsetLine(right_line_.getCoeff(), right_line_.getDegree(), offset_right);
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

  cv::Mat top_roi = right_lane_ROI_(cv::Rect(0, 0, TOPVIEW_COLS, TOPVIEW_ROWS / 5));
  top_roi.setTo(cv::Scalar(0, 0, 0));

  cv::bitwise_not(right_lane_ROI_, right_lane_ROI_);
  cv::bitwise_and(input_frame, right_lane_ROI_, output_frame);
}

void LaneDetector::ROILaneLeft(cv::Mat &input_frame, cv::Mat &output_frame)
{
  left_lane_ROI_ = cv::Mat::zeros(cv::Size(input_frame.cols, input_frame.rows), CV_8UC1);
  output_frame = input_frame.clone();
  float offset_center = 0.06;
  float offset_left = -0.08;
  if (starting_line_timeout_ > 0)
  {
    offset_center = 0.02;
    offset_left = -0.03;
    --starting_line_timeout_;
  }
  else
  {
    if (!left_line_.isExist() || left_line_.isShort())
      offset_left = -0.13;
    if (!center_line_.isExist() || center_line_.isShort())
      offset_center = 0.11;
  }
  
  std::vector<cv::Point2f> center_line_offset = createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), offset_center);
  std::vector<cv::Point2f> left_line_offset = createOffsetLine(left_line_.getCoeff(), left_line_.getDegree(), offset_left);
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

  cv::Mat top_roi = left_lane_ROI_(cv::Rect(0, 0, TOPVIEW_COLS, TOPVIEW_ROWS / 5));
  top_roi.setTo(cv::Scalar(0, 0, 0));

  cv::bitwise_not(left_lane_ROI_, left_lane_ROI_);
  cv::bitwise_and(input_frame, left_lane_ROI_, output_frame);
}

void LaneDetector::filterSmallLines()
{
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    //float sum = arcLength(lines_vector_converted_[i], false);
    float sum = lines_vector_converted_[i][lines_vector_converted_[i].size() - 1].x - lines_vector_converted_[i][0].x;
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

  lines_out_h_world_.clear();
  if (lines_out_h_.empty())
    return;

  for (int i = 0; i < lines_out_h_.size(); ++i)
  {
    cv::Point2f p = cv::Point2f(lines_out_h_[i][0], lines_out_h_[i][1]);
    lines_out_h_world_.push_back(p);
    p = cv::Point2f(lines_out_h_[i][2], lines_out_h_[i][3]);
    lines_out_h_world_.push_back(p);
  }
  cv::transform(lines_out_h_world_, lines_out_h_world_, topview2world_.rowRange(0, 2));
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

    p.y = getPolyY(left_line_.getCoeff(), x);
    aprox_lines_frame_coordinate_[0].push_back(p);

    p.y = getPolyY(center_line_.getCoeff(), x);
    aprox_lines_frame_coordinate_[1].push_back(p);

    p.y = getPolyY(right_line_.getCoeff(), x);
    aprox_lines_frame_coordinate_[2].push_back(p);
  }
  cv::transform(aprox_lines_frame_coordinate_[0], aprox_lines_frame_coordinate_[0], world2topview_.rowRange(0, 2));
  cv::transform(aprox_lines_frame_coordinate_[1], aprox_lines_frame_coordinate_[1], world2topview_.rowRange(0, 2));
  cv::transform(aprox_lines_frame_coordinate_[2], aprox_lines_frame_coordinate_[2], world2topview_.rowRange(0, 2));
}

void LaneDetector::initRecognizeLines()
{
  center_line_.clearPoints();
  right_line_.clearPoints();
  left_line_.clearPoints();
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
  if (min_index == -1)
  {
    center_line_.setExist(false);
    right_line_.setExist(false);
    left_line_.setExist(false);
    return;
  }

  center_line_.setExist(true);
  center_line_.addPoints(lines_vector_converted_[min_index]);

  min = homography_frame_.cols;
  min_index = -1;
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    if (lines_vector_converted_[i][0].y < center_line_.getPoints()[0].y &&
        lines_vector_converted_[i][0].x < 1 && cv::arcLength(lines_vector_converted_[i], false) > 0.2)
    {
      float distance_last = getDistance(center_line_.getPoints()[0], lines_vector_converted_[i][0]);
      float distance;
      for (int j = 1; j < lines_vector_converted_[i].size(); ++j)
      {
        distance = getDistance(center_line_.getPoints()[0], lines_vector_converted_[i][j]);
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
  if(min_index == -1)
  {
    right_line_.setExist(false);
  }
  else
  {
    right_line_.setExist(true);
    right_line_.addPoints(lines_vector_converted_[min_index]);
  }

  min = homography_frame_.cols;
  min_index = -1;
  for (int i = 0; i < lines_vector_converted_.size(); ++i)
  {
    if (lines_vector_converted_[i][0].y > center_line_.getPoints()[0].y &&
        lines_vector_converted_[i][0].x < 1 && cv::arcLength(lines_vector_converted_[i], false) > 0.2)
    {
      float distance_last = getDistance(center_line_.getPoints()[0], lines_vector_converted_[i][0]);
      float distance;
      for (int j = 1; j < lines_vector_converted_[i].size(); ++j)
      {
        distance = getDistance(center_line_.getPoints()[0], lines_vector_converted_[i][j]);
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
  if(min_index == -1)
  {
    left_line_.setExist(false);
  }
  else
  {
    left_line_.setExist(true);
    left_line_.addPoints(lines_vector_converted_[min_index]);
  }
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

void LaneDetector::linesApproximation()
{
  unsigned char l_state, c_state, r_state;

  int suitable_lines = 3;
  if (left_line_.isShort())
    --suitable_lines;
  if (right_line_.isShort())
    --suitable_lines;
  if (center_line_.isShort())
    --suitable_lines;

  if (init_imageCallback_)
  {
    left_line_.setDegree(2);
    center_line_.setDegree(2);
    right_line_.setDegree(2);
  }

  switch (suitable_lines)
  {
  case 3:
    // "l+  c+  r+"
    l_state = '+';
    c_state = '+';
    r_state = '+';

    if (init_imageCallback_)
    {
      left_line_.aprox();
      center_line_.aprox();
      right_line_.aprox();
    }
    else
    {
      left_line_.pfExecute();
      center_line_.pfExecute();
      right_line_.pfExecute();
    }
    break;

  case 2:
    if (left_line_.isShort())
    {
      if (init_imageCallback_)
      {
        center_line_.aprox();
        right_line_.aprox();
      }
      else
      {
        center_line_.pfExecute();
        right_line_.pfExecute();
      }
      
      left_line_.pfReset();
      if (!left_line_.isExist())
      {
        // "l-  c+  r+"
        l_state = '-';
        c_state = '+';
        r_state = '+';
        left_line_.setDegree(center_line_.getDegree());
        std::vector<float> tmp_coeff;
        if(polyfit(createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), left_lane_width_),
                                 left_line_.getDegree(), tmp_coeff))
          left_line_.setCoeff(tmp_coeff);
      }
      else
      {
        // l/  c+  r+"
        l_state = '/';
        c_state = '+';
        r_state = '+';
        left_line_.setDegree(center_line_.getDegree());
        adjust(center_line_, left_line_, true);
      }
    }
    else if (right_line_.isShort())
    {
      if (init_imageCallback_)
      {
        left_line_.aprox();
        center_line_.aprox();
      }
      else
      {
        left_line_.pfExecute();
        center_line_.pfExecute();
      }
      
      right_line_.pfReset();
      if (!right_line_.isExist())
      {
        // "l+  c+  r-"
        l_state = '+';
        c_state = '+';
        r_state = '-';
        right_line_.setDegree(center_line_.getDegree());
        std::vector<float> tmp_coeff;
        if(polyfit(createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), -1 * right_lane_width_),
                                 right_line_.getDegree(), tmp_coeff))
          right_line_.setCoeff(tmp_coeff);
      }
      else
      {
        // "l+  c+  r/"
        l_state = '+';
        c_state = '+';
        r_state = '/';
        right_line_.setDegree(center_line_.getDegree());
        adjust(center_line_, right_line_, false);
      }
    }
    else if (center_line_.isShort())
    {
      if (init_imageCallback_)
      {
        left_line_.aprox();
        right_line_.aprox();
      }
      else
      {
        left_line_.pfExecute();
        right_line_.pfExecute();
      }

      center_line_.pfReset();
      if (!center_line_.isExist())
      {
        // "l+  c-  r+"
        l_state = '+';
        c_state = '-';
        r_state = '+';
        center_line_.setDegree(right_line_.getDegree());
        std::vector<float> tmp_coeff;
        if(polyfit(createOffsetLine(right_line_.getCoeff(), right_line_.getDegree(), right_lane_width_),
                                 center_line_.getDegree(), tmp_coeff))
          center_line_.setCoeff(tmp_coeff);
      }
      else
      {
        // "l+  c/  r+"
        l_state = '+';
        c_state = '/';
        r_state = '+';
        center_line_.setDegree(right_line_.getDegree());
        adjust(right_line_, center_line_, true);
      }
    }
    break;

  case 1:
    if (!right_line_.isShort())
    {
      if (init_imageCallback_)
        right_line_.aprox();
      else
        right_line_.pfExecute();
      
      center_line_.pfReset();
      left_line_.pfReset();
      if (center_line_.isExist() || left_line_.isExist())
      {
        if (center_line_.isExist() && left_line_.isExist())
        {
          // "l/  c/  r+"
          l_state = '/';
          c_state = '/';
          r_state = '+';
          center_line_.setDegree(right_line_.getDegree());
          left_line_.setDegree(right_line_.getDegree());
          adjust(right_line_, center_line_, true);
          adjust(right_line_, left_line_, true);
        }
        else if (center_line_.isExist())
        {
          // "l-  c/  r+"
          l_state = '-';
          c_state = '/';
          r_state = '+';
          center_line_.setDegree(right_line_.getDegree());
          left_line_.setDegree(right_line_.getDegree());
          adjust(right_line_, center_line_, true);
          
          std::vector<float> tmp_coeff;
          if(polyfit(createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), left_lane_width_),
                                   left_line_.getDegree(), tmp_coeff))
            left_line_.setCoeff(tmp_coeff);
        }
        else
        {
          // "l/  c-  r+"
          l_state = '/';
          c_state = '-';
          r_state = '+';
          center_line_.setDegree(right_line_.getDegree());
          left_line_.setDegree(right_line_.getDegree());
          adjust(right_line_, left_line_, true);
          
          std::vector<float> tmp_coeff;
          if(polyfit(createOffsetLine(right_line_.getCoeff(), right_line_.getDegree(), right_lane_width_),
                                   center_line_.getDegree(), tmp_coeff))
            center_line_.setCoeff(tmp_coeff);
        }
      }
      else
      {
        // "l-  c-  r+"
        l_state = '-';
        c_state = '-';
        r_state = '+';
        center_line_.setDegree(right_line_.getDegree());
        left_line_.setDegree(right_line_.getDegree());
        
        std::vector<float> tmp_coeff;
        if(polyfit(createOffsetLine(right_line_.getCoeff(), right_line_.getDegree(), right_lane_width_),
                                 center_line_.getDegree(), tmp_coeff))
          center_line_.setCoeff(tmp_coeff);

        if(polyfit(createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), left_lane_width_),
                                 left_line_.getDegree(), tmp_coeff))
          left_line_.setCoeff(tmp_coeff);
      }
    }

    else if (!left_line_.isShort())
    {
      if (init_imageCallback_)
        left_line_.aprox();
      else
        left_line_.pfExecute();
      
      center_line_.pfReset();
      right_line_.pfReset();
      if (center_line_.isExist() || right_line_.isExist())
      {
        if (center_line_.isExist() && right_line_.isExist())
        {
          // "l+  c/  r/"
          l_state = '+';
          c_state = '/';
          r_state = '/';
          center_line_.setDegree(left_line_.getDegree());
          right_line_.setDegree(left_line_.getDegree());
          adjust(left_line_, center_line_, false);
          adjust(left_line_, right_line_, false);
        }
        else if (center_line_.isExist())
        {
          // "l+  c/  r-"
          l_state = '+';
          c_state = '/';
          r_state = '-';
          center_line_.setDegree(left_line_.getDegree());
          right_line_.setDegree(left_line_.getDegree());
          adjust(left_line_, center_line_, false);
          
          std::vector<float> tmp_coeff;
          if(polyfit(createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), -1 * right_lane_width_),
                                   right_line_.getDegree(), tmp_coeff))
            right_line_.setCoeff(tmp_coeff);
        }
        else
        {
          // "l+  c-  r/"
          l_state = '+';
          c_state = '-';
          r_state = '/';
          center_line_.setDegree(left_line_.getDegree());
          right_line_.setDegree(left_line_.getDegree());
          adjust(left_line_, right_line_, false);
          
          std::vector<float> tmp_coeff;
          if(polyfit(createOffsetLine(left_line_.getCoeff(), left_line_.getDegree(), -1 * left_lane_width_),
                                   center_line_.getDegree(), tmp_coeff))
            center_line_.setCoeff(tmp_coeff);
        }
      }
      else
      {
        // "l+  c-  r-"
        l_state = '+';
        c_state = '-';
        r_state = '-';
        center_line_.setDegree(left_line_.getDegree());
        right_line_.setDegree(left_line_.getDegree());
        std::vector<float> tmp_coeff;
        if(polyfit(createOffsetLine(left_line_.getCoeff(), left_line_.getDegree(), -1 * left_lane_width_),
                                 center_line_.getDegree(), tmp_coeff))
          center_line_.setCoeff(tmp_coeff);

        if(polyfit(createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), -1 * right_lane_width_),
                                 right_line_.getDegree(), tmp_coeff))
          right_line_.setCoeff(tmp_coeff);
      }
    }

    else if (!center_line_.isShort())
    {
      if (init_imageCallback_)
        center_line_.aprox();
      else
        center_line_.pfExecute();
      
      left_line_.pfReset();
      right_line_.pfReset();
      if (left_line_.isExist() || right_line_.isExist())
      {
        if (left_line_.isExist() && right_line_.isExist())
        {
          // "l/  c+  r/"
          l_state = '/';
          c_state = '+';
          r_state = '/';
          left_line_.setDegree(center_line_.getDegree());
          right_line_.setDegree(center_line_.getDegree());
          adjust(center_line_, left_line_, true);
          adjust(center_line_, right_line_, false);
        }
        else if (left_line_.isExist())
        {
          // "l/  c+  r-"
          l_state = '/';
          c_state = '+';
          r_state = '-';
          left_line_.setDegree(center_line_.getDegree());
          right_line_.setDegree(center_line_.getDegree());
          adjust(center_line_, left_line_, true);
          
          std::vector<float> tmp_coeff;
          if(polyfit(createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), -1 * right_lane_width_),
                                   right_line_.getDegree(), tmp_coeff))
            right_line_.setCoeff(tmp_coeff);
        }
        else
        {
          // "l-  c+  r/"
          l_state = '-';
          c_state = '+';
          r_state = '/';
          left_line_.setDegree(center_line_.getDegree());
          right_line_.setDegree(center_line_.getDegree());
          adjust(center_line_, right_line_, false);

          std::vector<float> tmp_coeff;
          if(polyfit(createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), left_lane_width_),
                                   left_line_.getDegree(), tmp_coeff))
            left_line_.setCoeff(tmp_coeff);
        }
      }
      else
      {
        // "l-  c+  r-"
        l_state = '-';
        c_state = '+';
        r_state = '-';
        left_line_.setDegree(center_line_.getDegree());
        right_line_.setDegree(center_line_.getDegree());
        
        std::vector<float> tmp_coeff;
        if(polyfit(createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), left_lane_width_),
                                 left_line_.getDegree(), tmp_coeff))
          left_line_.setCoeff(tmp_coeff);
        
        if(polyfit(createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), -1 * right_lane_width_),
                                 right_line_.getDegree(), tmp_coeff))
          right_line_.setCoeff(tmp_coeff);
      }
    }
    break;

  case 0:
    // "l-  c-  r-"
    l_state = '-';
    c_state = '-';
    r_state = '-';
    left_line_.pfReset();
    right_line_.pfReset();
    center_line_.pfReset();
    break;
  }

  if (debug_mode_)
  {
    std::cout << "--------aprox state---------" << std::endl;

    std::cout << "left_length   " << left_line_.getLength() << std::endl;
    std::cout << "center_length " << center_line_.getLength() << std::endl;
    std::cout << "right_length  " << right_line_.getLength() << std::endl
              << std::endl;

    std::cout << "left_degree   " << left_line_.getDegree() << std::endl;
    std::cout << "center_degree " << center_line_.getDegree() << std::endl;
    std::cout << "right_degree  " << right_line_.getDegree() << std::endl
              << std::endl;

    std::cout << "l" << l_state << "  c" << c_state << "  r" << r_state << std::endl;
  }
}

void LaneDetector::generatePoints()
{
  for (int k = 0; k < lines_vector_converted_.size(); ++k)
  {
    if (lines_vector_converted_[k].size() / cv::arcLength(lines_vector_converted_[k], false) > points_density_ * 2)
    {
      continue;
    }

    for (int i = 0; i < lines_vector_converted_[k].size() - 1; ++i)
    {
      float distance = getDistance(lines_vector_converted_[k][i],
                                   lines_vector_converted_[k][i + 1]);
      if (distance > 1 / points_density_)
      {
        int add = distance * points_density_;
        cv::Point2f p;
        float x1 = lines_vector_converted_[k][i].x;
        float y1 = lines_vector_converted_[k][i].y;
        float x_dif = (lines_vector_converted_[k][i + 1].x
                      - lines_vector_converted_[k][i].x) / (add + 1);
        float y_dif = (lines_vector_converted_[k][i + 1].y
                      - lines_vector_converted_[k][i].y) / (add + 1);
        for (int j = 0; j < add; ++j)
        {
          p.x = x1 + x_dif * (j + 1);
          p.y = y1 + y_dif * (j + 1);
          lines_vector_converted_[k].insert(lines_vector_converted_[k].begin() + i + 1, p);
          ++i;
        }
      }
    }
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
      cv::circle(visualization_frame, lines_vector_[i][j], 2, cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_AA);
    }
  }

  for (int i = 0; i < lines_vector_.size(); ++i)
  {
    for (int j = 1; j < lines_vector_[i].size(); ++j)
    {
      cv::line(visualization_frame, lines_vector_[i][j], lines_vector_[i][j - 1],
               cv::Scalar(color_set[i % 19][0], color_set[i % 19][1], color_set[i % 19][2]), 1);
    }
  }
}

void LaneDetector::LCRLinesDraw(cv::Mat &visualization_frame)
{
  visualization_frame = cv::Mat::zeros(visualization_frame.size(), CV_8UC3);
  std::vector<cv::Point2f> temp_vec;
  if (right_line_.isExist())
  {
    cv::transform(right_line_.getPoints(), temp_vec, world2topview_.rowRange(0, 2));

    for (int i = 0; i < temp_vec.size(); ++i)
    {
      cv::circle(visualization_frame, temp_vec[i], 2, cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_AA);
    }
    temp_vec.clear();
  }

  if (center_line_.isExist())
  {
    cv::transform(center_line_.getPoints(), temp_vec, world2topview_.rowRange(0, 2));

    for (int i = 0; i < temp_vec.size(); ++i)
    {
      cv::circle(visualization_frame, temp_vec[i], 2, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
    }
    temp_vec.clear();
  }

  if (left_line_.isExist())
  {
    cv::transform(left_line_.getPoints(), temp_vec, world2topview_.rowRange(0, 2));

    for (int i = 0; i < temp_vec.size(); ++i)
    {
      cv::circle(visualization_frame, temp_vec[i], 2, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    }
    temp_vec.clear();
  }
}

void LaneDetector::removeCar(cv::Mat &frame)
{
  cv::Mat car_mask = cv::Mat::zeros(cv::Size(frame.cols, frame.rows), CV_8UC1);
  cv::Point points[4];
  points[0] = cv::Point(310, TOPVIEW_ROWS);
  points[1] = cv::Point(335, TOPVIEW_ROWS);
  points[2] = cv::Point(330, 230);
  points[3] = cv::Point(310, 230);

  cv::fillConvexPoly(car_mask, points, 4, cv::Scalar(255, 255, 255));
  cv::bitwise_not(car_mask, car_mask);
  cv::bitwise_and(frame, car_mask, frame);
}

void LaneDetector::adjust(RoadLine &good_road_line,
                          RoadLine &short_road_line, bool left_offset)
{
  short_road_line.setDegree(good_road_line.getDegree());

  float step = 0.03;
  float offset = 0.2;
  int count = 0;
  if (!left_offset)
  {
    step *= -1;
    offset *= -1;
  }
  float last_cost = 9999999;  // init huge value
  std::vector<float> tmp_coeff;
  while (std::fabs(offset) < 1)
  {
    if (polyfit(createOffsetLine(good_road_line.getCoeff(), good_road_line.getDegree(), offset), short_road_line.getDegree(), tmp_coeff))
    {
      float current_cost = 0;
      for (int i = 0; i <= short_road_line.pointsSize(); ++i)
      {
        float aprox = getPolyY(tmp_coeff, short_road_line.getPoints()[i].x);
        current_cost += std::fabs(aprox - short_road_line.getPoints()[i].y);
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
  if (!polyfit(createOffsetLine(good_road_line.getCoeff(), good_road_line.getDegree(), offset), short_road_line.getDegree(), tmp_coeff))
  {
    std::vector<float> coeff = good_road_line.getCoeff();
    if (left_offset)
      coeff[0] += left_lane_width_;
    else
      coeff[0] -= right_lane_width_;
    short_road_line.setCoeff(coeff);
  }
  else
  {
    short_road_line.setCoeff(tmp_coeff);
  }

}

void LaneDetector::calcRoadWidth()
{
  debug_points_.clear();
  if (center_line_.getCoeff()[center_line_.getCoeff().size() - 1] == 0)
    return;
  cv::Point2f p;
  cv::Point2f p_ahead;
  p_ahead.x = 0.7;
  p_ahead.y = getPolyY(center_line_.getCoeff(), p_ahead.x);
  float deriative;
  if (center_line_.getCoeff().size() - 1 == 3)
    deriative = 3 * center_line_.getCoeff()[3] * pow(p_ahead.x, 2)
              + 2 * center_line_.getCoeff()[2] * p_ahead.x
              + center_line_.getCoeff()[1];
  else if(center_line_.getCoeff().size() - 1 == 2)
    deriative = 2 * center_line_.getCoeff()[2] * p_ahead.x
              + center_line_.getCoeff()[1];
  else
    deriative = center_line_.getCoeff()[1];

  float a_param_orthg = -1 / deriative;
  float b_param_orthg = p_ahead.y - a_param_orthg * p_ahead.x;

  // right lane
  float r_lane_width = 0;
  if (right_line_.isExist())
  {
    p.y = p_ahead.y;
    float step = -0.05;
    float last_dist = 9999999;  // init huge value
    while (true)
    {
      p.x = (p.y - b_param_orthg) / a_param_orthg;
      cv::Point2f p_aprox;
      p_aprox.x = p.x;
      p_aprox.y = getPolyY(right_line_.getCoeff(), p_aprox.x);
      r_lane_width = getDistance(p, p_aprox);
      if (r_lane_width - last_dist > 0)
      {
        if (debug_mode_)
        {
          debug_points_.clear();
          debug_points_.push_back(p_ahead);
          debug_points_.push_back(p_aprox);
        }
        r_lane_width = getDistance(p_ahead, p_aprox);
        break;
      }
      else
      {
        if (r_lane_width > 1)
        {
          break;
        }
        last_dist = r_lane_width;
        p.y += step;
      }
    }
    if (r_lane_width > 0.3 && r_lane_width < 0.5)
      right_lane_width_ = r_lane_width;
    else if (r_lane_width > 0.6)
    {
      ROS_INFO("Right line too far. Delete index");
      r_lane_width = 99;
      right_lane_width_ = 0.4;
      right_line_.setExist(false);
    }
  }

  // left lane
  float l_lane_width = 0;
  if (left_line_.isExist())
  {
    p.y = p_ahead.y;
    float step = 0.03;
    float last_dist = 9999999;  // init huge value

    while (true)
    {
      p.x = (p.y - b_param_orthg) / a_param_orthg;
      cv::Point2f p_aprox;
      p_aprox.x = p.x;
      p_aprox.y = getPolyY(left_line_.getCoeff(), p_aprox.x);
      l_lane_width = getDistance(p, p_aprox);
      if (l_lane_width - last_dist > 0)
      {
        if (debug_mode_)
        {
          debug_points_.push_back(p_ahead);
          debug_points_.push_back(p_aprox);
        }
        l_lane_width = getDistance(p_ahead, p_aprox);
        break;
      }
      else
      {
        if (l_lane_width > 1)
        {
          break;
        }
        last_dist = l_lane_width;
        p.y += step;
      }
    }
    if (l_lane_width > 0.3 && l_lane_width < 0.5)
      left_lane_width_ = l_lane_width;
    else if (l_lane_width > 0.6)
    {
      ROS_INFO("Left line too far. Delete index");
      l_lane_width = 99;
      left_lane_width_ = 0.4;
      left_line_.setExist(false);
    }
  }

  if (debug_mode_)
  {
    std::cout << std::endl;
    std::cout << "right width now: " << r_lane_width << std::endl;
    std::cout << "right_lane_width: " << right_lane_width_ << std::endl;
    std::cout << "left width now: " << l_lane_width << std::endl;
    std::cout << "left_lane_width: " << left_lane_width_ << std::endl;
  }
}

std::vector<cv::Point2f> LaneDetector::createOffsetLine(const std::vector<float> &coeff, const int &degree, float offset)
{
  std::vector<cv::Point2f> new_line;
  cv::Point2f p;
  for (float i = TOPVIEW_MIN_X - 0.11; i < TOPVIEW_MAX_X + 0.11; i += 0.1)
  {
    float deriative;
    switch (degree)
    {
    case 2:
      deriative = 2 * coeff[2] * i + coeff[1];
      break;
    case 3:
      deriative = 3 * coeff[3] * pow(i, 2) + 2 * coeff[2] * i + coeff[1];
      break;
    case 1:
      deriative = coeff[1];
      break;
    }

    float angle = atan(deriative);
    p.x = i - offset * sin(angle);
    p.y = getPolyY(coeff, i) + offset * cos(angle);
    new_line.push_back(p);
    if (p.y > TOPVIEW_MAX_Y || p.y < TOPVIEW_MIN_Y)
      break;
  }
  return new_line;
}

void LaneDetector::detectStartAndIntersectionLine()
{
  float detect_slope = 1.0;
  float detect_lenght = 0.15;

  // vectors
  std::vector<std::vector<cv::Point>> left_lines;
  std::vector<std::vector<cv::Point>> right_lines;
  std::vector<std::vector<cv::Point2f>> left_lines_converted;
  std::vector<std::vector<cv::Point2f>> right_lines_converted;

  float right_distance = -1000;
  float left_distance = -1000;

  cv::morphologyEx(left_lane_frame_, left_lane_frame_, 3, close_element_);
  cv::morphologyEx(right_lane_frame_, right_lane_frame_, 3, close_element_);
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
    if (proof_start_line_ == 3)
    {
      std_msgs::Float32 msg;
      msg.data = right_distance;
      starting_line_pub_.publish(msg);
      starting_line_timeout_ = 50;
    }
    else
    {
      ++proof_start_line_;
    }
    
  }
  else
  {
    if (proof_start_line_ > 0)
      --proof_start_line_;
    if (right_distance > 0)
    {
      if (proof_intersection_ == 3)
      {
        intersection_line_dist_ = right_distance;
        if(waiting_for_stabilize_ || intersection_)
        {
          std_msgs::Float32 msg;
          msg.data = intersection_line_dist_;
          intersection_pub_.publish(msg);
        }
      }
      else
      {
        intersection_line_dist_ = -1;
        ++proof_intersection_;
      }
    }
    else
    {
      if (proof_intersection_ > 0)
      {
        --proof_intersection_;
        intersection_line_dist_ = -1;
      }
    }
  }
}

void LaneDetector::drawParticles(int num)
{
  if(num > 19)
    num = 19;
  pf_vis_mat_ = homography_frame_.clone();
  if(pf_vis_mat_.type() == 0)
    cv::cvtColor(pf_vis_mat_, pf_vis_mat_, CV_GRAY2BGR);

  if(num < 2)
    return;

  std::vector<float> pf_coeff;
  std::vector<cv::Point2f> pf_cp;
  std::vector<cv::Point2f> pf_line;
  cv::Point2f p;
  for (int i = 0; i < num; ++i)
  {
    if (center_line_.isPFInitialized())
    {
      pf_line.clear();
      pf_cp.clear();
      pf_cp = center_line_.getParticleControlPoints(i);
      pf_coeff = center_line_.getParticleCoeff(i);
      for (float x = TOPVIEW_MIN_X; x < TOPVIEW_MAX_X; x += 0.03)
      {
        p.x = x;
        p.y = getPolyY(pf_coeff, x);
        pf_line.push_back(p);
      }
      cv::transform(pf_line, pf_line, world2topview_.rowRange(0, 2));
      cv::transform(pf_cp, pf_cp, world2topview_.rowRange(0, 2));
      for (int j = 1; j < pf_line.size(); ++j)
      {
        cv::line(pf_vis_mat_, pf_line[j - 1], pf_line[j], cv::Scalar(color_set[i][0], color_set[i][1], color_set[i][2]), 1.5, cv::LINE_AA);
      }
      for (int j = 0; j < pf_cp.size(); ++j)
      {
        cv::circle(pf_vis_mat_, pf_cp[j], 3, cv::Scalar(color_set[i][0], color_set[i][1], color_set[i][2]), cv::FILLED);
      }
    }

    if (left_line_.isPFInitialized())
    {
      pf_line.clear();
      pf_cp.clear();
      pf_cp = left_line_.getParticleControlPoints(i);
      pf_coeff = left_line_.getParticleCoeff(i);
      for (float x = TOPVIEW_MIN_X; x < TOPVIEW_MAX_X; x += 0.03)
      {
        p.x = x;
        p.y = getPolyY(pf_coeff, x);
        pf_line.push_back(p);
      }
      cv::transform(pf_line, pf_line, world2topview_.rowRange(0, 2));
      cv::transform(pf_cp, pf_cp, world2topview_.rowRange(0, 2));
      for (int j = 1; j < pf_line.size(); ++j)
      {
        cv::line(pf_vis_mat_, pf_line[j - 1], pf_line[j], cv::Scalar(color_set[i][0], color_set[i][1], color_set[i][2]), 1.5, cv::LINE_AA);
      }
      for (int j = 0; j < pf_cp.size(); ++j)
      {
        cv::circle(pf_vis_mat_, pf_cp[j], 3, cv::Scalar(color_set[i][0], color_set[i][1], color_set[i][2]), cv::FILLED);
      }
    }

    if (right_line_.isPFInitialized())
    {
      pf_line.clear();
      pf_cp.clear();
      pf_cp = right_line_.getParticleControlPoints(i);
      pf_coeff = right_line_.getParticleCoeff(i);
      for (float x = TOPVIEW_MIN_X; x < TOPVIEW_MAX_X; x += 0.03)
      {
        p.x = x;
        p.y = getPolyY(pf_coeff, x);
        pf_line.push_back(p);
      }
      cv::transform(pf_line, pf_line, world2topview_.rowRange(0, 2));
      cv::transform(pf_cp, pf_cp, world2topview_.rowRange(0, 2));
      for (int j = 1; j < pf_line.size(); ++j)
      {
        cv::line(pf_vis_mat_, pf_line[j - 1], pf_line[j], cv::Scalar(color_set[i][0], color_set[i][1], color_set[i][2]), 1.5, cv::LINE_AA);
      }
      for (int j = 0; j < pf_cp.size(); ++j)
      {
        cv::circle(pf_vis_mat_, pf_cp[j], 3, cv::Scalar(color_set[i][0], color_set[i][1], color_set[i][2]), cv::FILLED);
      }
    }
  }
}

void LaneDetector::createObstaclesMask()
{
  cv::threshold(homography_frame_, obstacles_mask_, obstacles_threshold_, 255, 0);

  cv::Mat erase_obstacles;
  morphologyEx(obstacles_mask_, erase_obstacles, cv::MORPH_TOPHAT , obstacle_element_);

  cv::bitwise_not(erase_obstacles, erase_obstacles);

  cv::bitwise_and(obstacles_mask_, erase_obstacles, obstacles_mask_);

  //cv::namedWindow("obstacles_mask_", cv::WINDOW_NORMAL);
  //cv::imshow("obstacles_mask_", obstacles_mask_);

  dilate( obstacles_mask_, obstacles_mask_, dilate_obst_element_ );
  //cv::namedWindow("obstacles_mask_++", cv::WINDOW_NORMAL);
  //cv::imshow("obstacles_mask_++", obstacles_mask_);

  cv::bitwise_not(obstacles_mask_, obstacles_mask_);
}

void LaneDetector::static_thresh_c_trackbar(int v, void *ptr)
{
  // resolve 'this':
  LaneDetector *that = (LaneDetector*)ptr;
  that->on_thresh_c_trackbar(v);
}

void LaneDetector::on_thresh_c_trackbar(int v)
{
 threshold_c_ = thresh_c_tune_temp_ - 50;
}

void LaneDetector::tuneParams(const ros::TimerEvent &time)
{
  cv::namedWindow("trackbars", cv::WINDOW_NORMAL);
  cv::createTrackbar("obstacle_thresh ", "trackbars", &obstacles_threshold_, 255);
  cv::createTrackbar("threshold_c [-50,50]", "trackbars", &thresh_c_tune_temp_, 100, static_thresh_c_trackbar, this);

  cv::Mat homography_gray;
  homography(current_frame_, homography_gray);
  cv::Mat test_obstacle_mat;
  cv::threshold(homography_gray, test_obstacle_mat, obstacles_threshold_, 255, 0);
  cv::namedWindow("test_obstacle_mat", cv::WINDOW_NORMAL);
  cv::imshow("test_obstacle_mat", test_obstacle_mat);

  cv::Mat test_threshold_c_mat;
  cv::adaptiveThreshold(homography_gray, test_threshold_c_mat, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                        CV_THRESH_BINARY, treshold_block_size_, threshold_c_);

  cv::namedWindow("test_threshold_c_mat", cv::WINDOW_NORMAL);
  cv::imshow("test_threshold_c_mat", test_threshold_c_mat);

  cv::waitKey(1);
}

bool LaneDetector::isIntersection()
{
  if (lines_out_h_world_.empty())
  {
    if (intersection_)
    {
      waiting_for_stabilize_ = true;
      ROS_INFO("waiting_for_stabilize is true");
    }
      
    intersection_ = false;
    //center_line_.setDegree(2);
    //right_line_.setDegree(2);
    //left_line_.setDegree(2);
    return false;
  }

  bool left_intersection = false;
  bool right_intersection = false;
  float right_lenght = 0;
  int min_right_index = -1;
  int min_left_index = -1;

  // check right line
  if (right_line_.isExist())
  {
    for (int i = 0; i < lines_out_h_world_.size(); i += 2)
    {
      cv::Point2f p_center = cv::Point2f(lines_out_h_world_[i].x, getPolyY(center_line_.getCoeff(), lines_out_h_world_[i].x));
      if (p_center.y < lines_out_h_world_[i].y && p_center.y < lines_out_h_world_[i + 1].y)
        continue;

      float dist1 = getDistance(lines_out_h_world_[i], p_center);
      float dist2 = getDistance(lines_out_h_world_[i + 1], p_center);

      cv::Point2f p_inner;
      if(dist1 < dist2)
        p_inner = lines_out_h_world_[i];
      else
        p_inner = lines_out_h_world_[i + 1];

      if (p_inner.x - right_line_.getPoints()[0].x < 0.025)
        continue;

      float min_dist = 99999; // init huge value
      for (int j = 0; j < right_line_.pointsSize(); ++j)
      {
        float dist = getDistance(p_inner, right_line_.getPoints()[j]);
        if (dist < min_dist)
        {
          min_dist = dist;
          min_right_index = j;
        }
      }

      if (min_dist < isec_line_close_)
      {
        isec_min_dist_points_.push_back(right_line_.getPoints()[min_right_index]);
        float a1 = atan2(lines_out_h_world_[i].y - lines_out_h_world_[i + 1].y,
                         lines_out_h_world_[i].x - lines_out_h_world_[i + 1].x);
        cv::Point2f p_line = right_line_.getPointNextToBottom(0.1);
        float a2 = atan2(p_line.y - right_line_.getPoints()[0].y,
                         p_line.x - right_line_.getPoints()[0].x);
        float a = std::abs(std::abs(a1 - a2) - 1.57);

        if (a > isec_angle_diff_)
          continue;
        if (debug_mode_)
        {
          cv::Point2f p = cv::Point2f(p_line.x, p_line.y);
          isec_debug_points_.push_back(p);
          isec_debug_points_.push_back(right_line_.getPoints()[0]);
          isec_debug_points_.push_back(lines_out_h_world_[i]);
          isec_debug_points_.push_back(lines_out_h_world_[i + 1]);
        }
        float d = getDistance(lines_out_h_world_[i], lines_out_h_world_[i + 1]);
        if(d > right_lenght)
          right_lenght = d;
        right_intersection = true;
        break;
      }
    }
  }

  // check left line
  if (left_line_.isExist())
  {
    for (int i = 0; i < lines_out_h_world_.size(); i += 2)
    {
      cv::Point2f p_center = cv::Point2f(lines_out_h_world_[i].x, getPolyY(center_line_.getCoeff(), lines_out_h_world_[i].x));
      if (p_center.y > lines_out_h_world_[i].y && p_center.y > lines_out_h_world_[i + 1].y)
        continue;

      float dist1 = getDistance(lines_out_h_world_[i], p_center);
      float dist2 = getDistance(lines_out_h_world_[i + 1], p_center);

      cv::Point2f p_inner;
      if(dist1 < dist2)
        p_inner = lines_out_h_world_[i];
      else
        p_inner = lines_out_h_world_[i + 1];

      if (p_inner.x - left_line_.getPoints()[0].x < 0.025)
        continue;

      float min_dist = 99999; // init huge value
      for (int j = 0; j < left_line_.pointsSize(); ++j)
      {
        float dist = getDistance(p_inner, left_line_.getPoints()[j]);
        if (dist < min_dist)
        {
          min_dist = dist;
          min_left_index = j;
        }
      }

      if (min_dist < isec_line_close_)
      {
        isec_min_dist_points_.push_back(left_line_.getPoints()[min_left_index]);
        float a1 = atan2(lines_out_h_world_[i].y - lines_out_h_world_[i + 1].y,
                         lines_out_h_world_[i].x - lines_out_h_world_[i + 1].x);
        cv::Point2f p_line = left_line_.getPointNextToBottom(0.1);
        float a2 = atan2(p_line.y - left_line_.getPoints()[0].y,
                         p_line.x - left_line_.getPoints()[0].x);
        float a = std::abs(std::abs(a1 - a2) - 1.57);

        if (a > isec_angle_diff_)
          continue;
        if (debug_mode_)
        {
          cv::Point2f p = cv::Point2f(p_line.x, p_line.y);
          isec_debug_points_.push_back(p);
          isec_debug_points_.push_back(left_line_.getPoints()[0]);
          isec_debug_points_.push_back(lines_out_h_world_[i]);
          isec_debug_points_.push_back(lines_out_h_world_[i + 1]);
        }
        left_intersection = true;
        break;
      }
    }
  }
  if(right_lenght > 0.5)
    left_intersection = true; 
  
  int index_on_merge = 0;
  if (center_line_.isExist())
  {
    index_on_merge = center_line_.getIndexOnMerge();
  }

  if(left_intersection && right_intersection && center_line_.isExist() && 
     (center_line_.getPoints()[index_on_merge].x - center_line_.getPoints()[0].x > 0.15))
  {
    if (center_line_.getPoints()[0].x > ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 2))
    {
      if (intersection_)
        waiting_for_stabilize_ = true;
      intersection_ = false;
      //center_line_.setDegree(2);
      //right_line_.setDegree(2);
      //left_line_.setDegree(2);
      return false;
    }
    intersection_ = true;
    waiting_for_stabilize_ = false;
    ROS_INFO_THROTTLE(2, "INTERSECTION");

    center_line_.setDegree(1);
    right_line_.setDegree(1);
    left_line_.setDegree(1);
    center_line_.reducePointsToStraight(index_on_merge);
    center_line_.aprox();
    if (right_line_.isExist())
    {
      right_line_.reduceTopPoints(0.5);
      adjust(center_line_, right_line_, false);
    }
    else
    {
      right_line_.setDegree(center_line_.getDegree());
      std::vector<float> tmp_coeff;
      polyfit(createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), -1 * right_lane_width_),
                               right_line_.getDegree(), tmp_coeff);
      right_line_.setCoeff(tmp_coeff);
    }
    
    if (left_line_.isExist())
    {
      left_line_.reduceTopPoints(0.5);
      adjust(center_line_, left_line_, true);
    }
    else
    {
      left_line_.setDegree(center_line_.getDegree());
      std::vector<float> tmp_coeff;
      polyfit(createOffsetLine(center_line_.getCoeff(), center_line_.getDegree(), left_lane_width_),
                               left_line_.getDegree(), tmp_coeff);
      left_line_.setCoeff(tmp_coeff);
    }

    center_line_.pfReset();
    right_line_.pfReset();
    left_line_.pfReset();
    return true;
  }
  else
  {
    if (intersection_)
    {
      waiting_for_stabilize_ = true;
      ROS_INFO("waiting_for_stabilize is true");
    }
    intersection_ = false;
    //center_line_.setDegree(2);
    //right_line_.setDegree(2);
    //left_line_.setDegree(2);
    return false;
  }
}

void LaneDetector::drawIntersection()
{
  if(outside_road_.type() == 0)
    cv::cvtColor(outside_road_, outside_road_, CV_GRAY2BGR);
  for (size_t i = 0; i < lines_out_h_.size(); ++i)
  {
    cv::Vec4i l = lines_out_h_[i];
    cv::line(outside_road_, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
  }

  if(!isec_min_dist_points_.empty())
  {
    cv::transform(isec_min_dist_points_, isec_min_dist_points_, world2topview_.rowRange(0, 2));
    for (size_t i = 0; i < isec_min_dist_points_.size(); i ++)
    {
      cv::circle(outside_road_, isec_min_dist_points_[i], 2, cv::Scalar(255, 255, 0), cv::FILLED);
    }
    isec_min_dist_points_.clear();
  }

  if (isec_debug_points_.empty())
    return;

  cv::transform(isec_debug_points_, isec_debug_points_, world2topview_.rowRange(0, 2));
  for (size_t i = 0; i < isec_debug_points_.size(); i += 2)
  {
    cv::Vec4i l = lines_out_h_[i];
    cv::line(outside_road_, isec_debug_points_[i], isec_debug_points_[i + 1], cv::Scalar(0,255,0), 1, cv::LINE_AA);
  }
  isec_debug_points_.clear();
}