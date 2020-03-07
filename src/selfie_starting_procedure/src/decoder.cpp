/***Copyright ( c) 2019, KNR Selfie*
 * This code is licensed under BSD license (see LICENSE for details)
 **/
#include "selfie_starting_procedure/qr_decoder.h"

QrDecoder::QrDecoder(const ros::NodeHandle &nh, const ros::NodeHandle &pnh): nh_(nh), pnh_(pnh)
{
  zbar_scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  zbar_scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
  gate_open_pub_ = nh_.advertise<std_msgs::Empty>("qr_gate_open", 1);
  pnh_.param<float>("min_detect_rate", min_detect_rate_, 0.4);
  pnh_.param<int>("iterations_to_vaild", iterations_to_vaild_, 2);
  pnh_.param("visualize", visualize_, false);

  ROS_INFO("min_detect_rate: %.2f", min_detect_rate_);
  ROS_INFO("iterations_to_vaild: %d", iterations_to_vaild_);

  start_serv_ = nh_.advertiseService("startQrSearch", &QrDecoder::startSearching, this);
  stop_serv_ = nh_.advertiseService("stopQrSearch", &QrDecoder::stopSearching, this);

  ROS_INFO("QrDetector initialized");
}

bool QrDecoder::startSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp)
{
  image_sub_ = nh_.subscribe("image_rect", 1, &QrDecoder::imageRectCallback, this);
  count_valid_iterations_= 0;
  init_ = false;
  count_valid_iterations_= 0;
  ROS_INFO("QrDetector start searching");
  return true;
}

bool QrDecoder::stopSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp)
{
  rate_timer_.stop();
  image_sub_.shutdown();
  ROS_INFO("QrDetector stop searching");
  return true;
}

void QrDecoder::imageRectCallback(const sensor_msgs::Image::ConstPtr img)
{
  this->decodeImage(cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC1));
}

void QrDecoder::decodeImage(const cv_bridge::CvImagePtr raw_img)
{
  cv::Mat input = raw_img->image;
  cv::Mat output;

  cv::Rect region_of_interest = cv::Rect(input.cols / 3, 0, input.cols / 3, input.rows / 2);
  cv::Mat image_roi = input(region_of_interest);
  cv::GaussianBlur(image_roi, image_roi, cv::Size(3,3), 0);
  cv::resize(image_roi, image_roi, image_roi.size(), 2, 2, cv::INTER_LINEAR);

  if (!init_)
  {
    cv::equalizeHist(image_roi,image_roi);
    cv::GaussianBlur(image_roi, output, cv::Size(0, 0), 3);
    cv::addWeighted(image_roi, 1.5, output, -1.0, 0, output);
  }
  else
  {
    cv::GaussianBlur(image_roi, output, cv::Size(0, 0), 3);
    cv::addWeighted(image_roi, 1.5, output, -1.0, 0, output);

    cv::warpPerspective(output, output, M_, cv::Size(trans_width_, trans_height_));

    cv::equalizeHist(output,output);
  }

  if(visualize_)
  {
    cv::namedWindow("raw",cv::WINDOW_NORMAL);
    cv::imshow("raw",image_roi);

    cv::namedWindow("qr",cv::WINDOW_NORMAL);
    cv::imshow("qr",output);
    cv::waitKey(1);
  }

  int width = output.cols;
  int height = output.rows;

  zbar::Image img(width, height, "Y800",output.data, width * height);
  zbar_scanner_.scan(img);

  ++count_frame_;
  if (boost::algorithm::any_of(img.symbol_begin(), img.symbol_end(),
   [](zbar::Symbol sym){return sym.get_data() == "STOP";}))
  {
    ++count_bar_;
    if(!init_)
    {
      std::vector<cv::Point> v_points;
      for(int i = 0; i< img.symbol_begin()->get_location_size(); i++)
      {
        cv::Point p = cv::Point(img.symbol_begin()->get_location_x(i), img.symbol_begin()->get_location_y(i));
        v_points.push_back(p);
      }
      cv::RotatedRect rr_ = cv::minAreaRect(v_points);
      
      int offset = 0;
      for (int i = 0; i < v_points.size(); ++i)
      {
        if (v_points[i].x < rr_.center.x)
        {
          if (v_points[i].y < rr_.center.y)
          {
            tl_ = v_points[i];
            tl_.x += offset;
            tl_.y += offset;
          }
          else
          {
            bl_ = v_points[i];
            bl_.x += offset;
            bl_.y -= offset;
          }
        }
        else
        {
          if (v_points[i].y < rr_.center.y)
          {
            tr_ = v_points[i];
            tr_.x -= offset;
            tr_.y += offset;
          }
          else
          {
            br_ = v_points[i];
            br_.x -= offset;
            br_.y -= offset;
          }
        }
      }
      std::vector<cv::Point2f> rect;
      rect.emplace_back(tl_);
      rect.emplace_back(tr_);
      rect.emplace_back(br_);
      rect.emplace_back(bl_);
      float widthA = sqrt((tl_.x - tr_.x) * (tl_.x - tr_.x) + (tl_.y - tr_.y) * (tl_.y - tr_.y));
      float widthB = sqrt((bl_.x - br_.x) * (bl_.x - br_.x) + (bl_.y - br_.y) * (bl_.y - br_.y));
      float heightA = sqrt((tl_.x - bl_.x) * (tl_.x - bl_.x) + (tl_.y - bl_.y) * (tl_.y - bl_.y));
      float heightB = sqrt((tr_.x - br_.x) * (tr_.x - br_.x) + (tr_.y - br_.y) * (tr_.y - br_.y));
      trans_width_ = std::max(widthA, widthB);
      trans_height_ = std::max(heightA, heightB);

      std::vector<cv::Point2f> dst;
      dst.emplace_back(0,0);
      dst.emplace_back(trans_width_ - 1 ,0);
      dst.emplace_back(trans_width_ - 1 , trans_height_ - 1);
      dst.emplace_back(0,trans_height_ - 1);

      M_ = cv::getPerspectiveTransform(rect, dst);

      ROS_INFO("QrDetector - QR found");
      init_ = true;
      rate_timer_ = nh_.createTimer(ros::Duration(1), &QrDecoder::calcRate, this);
    }
  }
}

void QrDecoder::calcRate(const ros::TimerEvent &time)
{
  if (count_frame_ != 0)
  {
    detect_rate_ = count_bar_ / float(count_frame_);
  }
  else
  {
    detect_rate_ = 0;
  }
  count_bar_ = 0;
  count_frame_ = 0;
  if (count_valid_iterations_ < iterations_to_vaild_)
  {
    if (detect_rate_ < min_detect_rate_)
    {
      rate_timer_.stop();
      init_ = false;
      count_valid_iterations_= 0;
      ROS_INFO("QrDetector - detection not valid");
      return;
    }
    ++count_valid_iterations_;
    if (count_valid_iterations_ == iterations_to_vaild_)
    {
      ROS_INFO("QrDetector - detection valid");
    }
    return;
  }
  
  ROS_INFO("QrDetection rate: %.3f", detect_rate_);
  if (detect_rate_ < min_detect_rate_)
  {
    ROS_INFO("QrDetector - gate opened");
    gate_open_pub_.publish(std_msgs::Empty());
    image_sub_.shutdown();

    if(visualize_)
    {
      cv::destroyAllWindows();
    }
    rate_timer_.stop();
  }
}
