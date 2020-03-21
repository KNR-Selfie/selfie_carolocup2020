#ifndef QR_DECODER_H 
#define QR_DECODER_H

#include <ros/ros.h>
#include <zbar.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <list>

class QrDecoder
{
  ros::NodeHandle nh_,pnh_;
  ros::Subscriber image_sub_; 
  ros::ServiceServer start_serv_;
  ros::ServiceServer stop_serv_;
  ros::Publisher gate_open_pub_;

  zbar::ImageScanner zbar_scanner_;
  zbar::Image zbar_image_;

  bool init_{false};

  cv::Point tr_;
  cv::Point tl_;
  cv::Point br_;
  cv::Point bl_;

  float trans_width_;
  float trans_height_;
  cv::Mat M_;

  bool visualize_ {false};
  float min_detect_rate_ {0.2};
  int iterations_to_vaild_ {2};
  float detect_rate_ {1.0};

  int count_frame_{0};
  int count_bar_{0};
  int count_valid_iterations_{0};

  ros::Timer rate_timer_;
  cv_bridge::CvImagePtr cv_ptr;

  bool startSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp);
  bool stopSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp);

  void imageRectCallback(const sensor_msgs::Image::ConstPtr msg);
  void decodeImage(const cv_bridge::CvImagePtr img);
  void calcRate(const ros::TimerEvent &time);
public:
  QrDecoder(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
};
#endif