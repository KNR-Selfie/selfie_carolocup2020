/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#ifndef SELFIE_PERCEPTION_LANE_DETECTOR_H
#define SELFIE_PERCEPTION_LANE_DETECTOR_H
#pragma once
#define BOOST_UBLAS_TYPE_CHECK 0

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <selfie_msgs/RoadMarkings.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <std_srvs/Empty.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>

struct RoadLine
{
  int index     {-1};
  int degree    {2};
  float length  {0};
  bool is_short {false};
  std::vector<float> coeff;

  RoadLine()
  {
    float empty_coeff = 0;
    coeff.push_back(empty_coeff);
    coeff.push_back(empty_coeff);
    coeff.push_back(empty_coeff);
  }
};

class LaneDetector
{
  public:
  LaneDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
  ~LaneDetector();
  bool resetVisionCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool init();

  private:
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
  cv::Mat current_frame_;
  cv::Mat binary_frame_;
  cv::Mat binary_cut_frame_;
  cv::Mat dynamic_mask_;
  cv::Mat masked_frame_;
  cv::Mat left_lane_ROI_;
  cv::Mat left_lane_frame_;
  cv::Mat right_lane_ROI_;
  cv::Mat right_lane_frame_;
  cv::Mat homography_frame_;
  cv::Mat debug_frame_;
  cv::Mat hom_cut_mask_;

  std::vector<std::vector<cv::Point> > lines_vector_;
  std::vector<std::vector<cv::Point2f> > lines_vector_converted_;
  std::vector<std::vector<cv::Point2f> > aprox_lines_frame_coordinate_;

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
  float getAproxY(std::vector<float> coeff, float x);
  void convertApproxToFrameCoordinate();
  void initRecognizeLines();
  void linesApproximation();
  void calcRoadLinesParams();
  void removeCar(cv::Mat &frame);
  void addBottomPoint();
  bool polyfit(std::vector<cv::Point2f> line, RoadLine &road_line);
  bool polyfit(RoadLine &road_line);
  void adjust(RoadLine &good_road_line, RoadLine &short_road_line, bool left_offset);
  void calcRoadWidth();
  void generatePoints();
  void removeHorizontalLines();
  std::vector<cv::Point2f> createOffsetLine(RoadLine &road_line, float offset);
  void detectStartAndIntersectionLine();

  // visualization
  sensor_msgs::PointCloud points_cloud_;
  ros::Publisher points_cloud_pub_;
  ros::Publisher aprox_visualization_pub_;
  void pointsRVIZVisualization();
  void aproxRVIZVisualization();
  void drawAproxOnHomography();
  void lanesVectorVisualization(cv::Mat &visualization_frame);

  bool starting_line_           {false};
  bool init_imageCallback_      {true};
  float min_length_search_line_ {0.10};
  float max_delta_y_lane_       {0.08};
  float min_length_to_2aprox_   {0.56};
  float left_lane_width_        {0.4};
  float right_lane_width_       {0.4};

// parameterized
  std::string config_file_      {""};
  bool debug_mode_              {false};
  bool hom_cut_tune_mode_       {false};
  float max_mid_line_distance_  {0.12};
  float max_mid_line_gap_       {0.9};
  float nominal_center_line_Y_  {0.2};
  float points_density_         {15};
  int treshold_block_size_      {3};
  float real_window_size_       {0.1};
  int threshold_c_              {-40};
  int hom_cut_l_x_              {0};
  int hom_cut_l_y_              {0};
  int hom_cut_r_x_              {0};
  int hom_cut_r_y_              {0};
};

#endif  //  SELFIE_PERCEPTION_LANE_DETECTOR_H
