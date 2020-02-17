/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef SELFIE_PERCEPTION_LANE_DETECTOR_H
#define SELFIE_PERCEPTION_LANE_DETECTOR_H
#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <std_srvs/Empty.h>
#include <selfie_msgs/RoadMarkings.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <algorithm>
#include <stdexcept>

#include <selfie_perception/road_line.h>
#include <selfie_perception/polynomials.h>
#include <selfie_perception/definitions.h>

class LaneDetector
{
  public:
  LaneDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~LaneDetector();
  bool resetVisionCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool init();

  private:
  ros::Timer tune_timer_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher lanes_pub_;
  ros::Publisher intersection_pub_;
  ros::Publisher starting_line_pub_;

  cv::Size topview_size_;
  cv::Mat world2cam_;
  cv::Mat topview2world_;
  cv::Mat topview2cam_;
  cv::Mat world2topview_;

  cv::Mat kernel_v_;
  cv::Mat kernel_h_;
  cv::Mat dilate_element_;
  cv::Mat close_element_;
  cv::Mat obstacle_element_;
  cv::Mat dilate_obst_element_;
  cv::Mat current_frame_;
  cv::Mat binary_frame_;
  cv::Mat dynamic_mask_;
  cv::Mat masked_frame_;
  cv::Mat left_lane_ROI_;
  cv::Mat left_lane_frame_;
  cv::Mat right_lane_ROI_;
  cv::Mat right_lane_frame_;
  cv::Mat homography_frame_;
  cv::Mat homography_masked_frame_;
  cv::Mat debug_frame_;
  cv::Mat hom_cut_mask_;
  cv::Mat hom_cut_mask_inv_;
  cv::Mat obstacles_mask_;
  cv::Mat pf_vis_mat_;
  cv::Mat outside_road_;

  std::vector<std::vector<cv::Point> > lines_vector_;
  std::vector<std::vector<cv::Point2f> > lines_vector_converted_;
  std::vector<std::vector<cv::Point2f> > aprox_lines_frame_coordinate_;
  std::vector<cv::Vec4i> lines_out_h_;
  std::vector<cv::Point2f> lines_out_h_world_;

  std::vector<cv::Point2f> debug_points_;
  std::vector<cv::Point2f> isec_debug_points_;
  std::vector<cv::Point2f> isec_min_dist_points_;

  RoadLine left_line_;
  RoadLine center_line_;
  RoadLine right_line_;

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void computeTopView();
  void openCVVisualization();
  void mergeMiddleLines();
  void quickSortLinesY(int left, int right);
  void quickSortPointsY(std::vector<cv::Point> &vector_in, int left, int right);
  float getDistance(cv::Point2f p1, cv::Point2f p2);
  void recognizeLines();
  void publishMarkings();
  void detectLines(cv::Mat &input_frame, std::vector<std::vector<cv::Point> > &output_lanes);
  void drawPoints(cv::Mat &frame);
  void homography(cv::Mat input_frame, cv::Mat &homography_frame);
  void getParams();
  void printInfoParams();
  void dynamicMask(cv::Mat &input_frame, cv::Mat &output_frame);
  void ROILaneLeft(cv::Mat &input_frame, cv::Mat &output_frame);
  void ROILaneRight(cv::Mat &input_frame, cv::Mat &output_frame);
  void filterSmallLines();
  void convertCoordinates();
  void convertApproxToFrameCoordinate();
  void initRecognizeLines();
  void linesApproximation();
  void calcRoadLinesParams();
  void removeCar(cv::Mat &frame);
  void adjust(RoadLine &good_road_line, RoadLine &short_road_line, bool left_offset);
  void calcRoadWidth();
  void generatePoints();
  std::vector<cv::Point2f> createOffsetLine(const std::vector<float> &coeff, const int &degree, float offset);
  void detectStartAndIntersectionLine();
  void recognizeLinesNew();
  void LCRLinesDraw(cv::Mat &visualization_frame);
  float findMinPointToParabola(cv::Point2f p, std::vector<float> coeff);
  void createObstaclesMask();
  void tuneParams(const ros::TimerEvent &time);
  static void static_thresh_c_trackbar(int v, void *ptr);
  void on_thresh_c_trackbar(int v);
  bool isIntersection();
  void drawIntersection();

  // visualization
  sensor_msgs::PointCloud points_cloud_;
  ros::Publisher points_cloud_pub_;
  void pointsRVIZVisualization();
  void drawAproxOnHomographyMasked();
  void lanesVectorVisualization(cv::Mat &visualization_frame);
  void drawParticles(int num);

  int starting_line_timeout_    {0};
  bool init_imageCallback_      {true};
  float min_length_search_line_ {0.05};
  float max_delta_y_lane_       {0.08};
  float min_length_to_2aprox_   {0.56};
  float left_lane_width_        {0.4};
  float right_lane_width_       {0.4};
  int proof_intersection_       {0};
  int proof_start_line_         {0};
  float intersection_line_dist_ {-1};
  bool intersection_            {false};
  bool waiting_for_stabilize_   {false};

// parameterized
  std::string config_file_      {""};
  std::string hom_cut_file_     {""};
  bool debug_mode_              {false};
  bool tune_params_mode_        {false};
  int thresh_c_tune_temp_       {false};

  float max_mid_line_distance_  {0.15};
  float max_mid_line_gap_       {0.38};
  float nominal_center_line_Y_  {0.2};
  float points_density_         {15};

  int treshold_block_size_      {3};
  float real_window_size_       {0.1};
  int threshold_c_              {-40};

  float obstacle_window_size_   {0.09};
  int obstacles_threshold_      {100};

  int pf_num_samples_           {50};
  int pf_num_points_            {3};
  float pf_std_min_             {0.005};
  float pf_std_max_             {0.02};
  int pf_num_samples_vis_       {20};

  double isec_HL_dist_res_      {1};
  double isec_HL_angle_res_     {CV_PI / 180};
  int isec_HL_thresh_           {25};
  double isec_HL_min_length_    {25};
  int isec_HL_max_gap_          {40};

  float isec_line_close_        {0.15};
  float isec_angle_diff_        {20 / 57.2957795};

  int color_set[20][3] =
  {
    {0 , 0 , 255},
    {255 , 0, 0},
    {0 , 255, 0},
    {255 , 255, 0},
    {0 , 255, 255},
    {255 , 0, 255},
    {255 , 255, 255},
    {0 , 0, 128},
    {128 , 0, 0},
    {0 , 128, 0},
    {128 , 128, 0},
    {0 , 128, 128},
    {128 , 0, 128},
    {128 , 128, 128},
    {196 , 228, 255},
    {30 , 105, 210},
    {143 , 143, 188},
    {222 , 196, 176},
    {75 , 0, 130},
    {5 , 128, 255}
  };
};

#endif  //  SELFIE_PERCEPTION_LANE_DETECTOR_H
