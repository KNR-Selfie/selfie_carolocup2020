#include "selfie_starting_procedure/qr_decoder.h"

QrDecoder::QrDecoder(const ros::NodeHandle &nh,const ros::NodeHandle &pnh):nh_(nh),pnh_(pnh)
{
    zbarScanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1 );
    gateOpenPub_ = nh_.advertise<std_msgs::Empty>("qr_gate_open",1);

    pnh_.param<float>("qr_invisible_time_thresh",qrInvisibleTimeThresh_,1.f);
    timer_ = nh_.createTimer(ros::Duration(qrInvisibleTimeThresh_) ,&QrDecoder::timerCallback, this, true, false);//oneshot, autostart
    startServ_ = nh_.advertiseService("startQrSearch", &QrDecoder::startSearching, this);
}
bool QrDecoder::startSearching(std_srvs::Empty::Request &rq, std_srvs::Empty::Response &rp)
{
    imageSub_ = nh_.subscribe("image_rect",1, &QrDecoder::imageRectCallback, this);
    timer_.start();
    return true;
}

void QrDecoder::timerCallback(const ros::TimerEvent &e)
{
    gateOpenPub_.publish(std_msgs::Empty());
    imageSub_.shutdown();
}

void QrDecoder::imageRectCallback(const sensor_msgs::Image::ConstPtr img)
{
    this->decodeImage(cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8));
}

void QrDecoder::decodeImage(const cv_bridge::CvImagePtr raw_img)
{
    cv_ptr = raw_img;
    cv::cvtColor(cv_ptr->image,cv_ptr->image,CV_BGR2GRAY);

    int width = cv_ptr->image.cols;
    int height = cv_ptr->image.rows;

    uchar *raw = (uchar *)cv_ptr->image.data;

    zbar::Image img(width, height, "Y800", raw, width * height);

    zbarScanner_.scan(img);
    if(boost::algorithm::any_of(img.symbol_begin(),img.symbol_end(),[](zbar::Symbol sym){return sym.get_data() == "STOP";})) 
    {
        resetTimer();
    }
}

void QrDecoder::resetTimer()
{
    timer_.setPeriod(ros::Duration(qrInvisibleTimeThresh_), true);//reset
    std::cout<<"timer reset"<<std::endl;
}
