#include <selfie_perception/road_line.h>

RoadLine::RoadLine()
{
  coeff_.clear();
  float empty_coeff = 0;
  coeff_.push_back(empty_coeff);
  coeff_.push_back(empty_coeff);
  coeff_.push_back(empty_coeff);
}

void RoadLine::pfSetup(int num_particles, int num_control_points, float std)
{
  pf_num_particles_ = num_particles;
  pf_num_points_ = num_control_points;
  pf_std_ = std;

  pf_.setNumParticles(num_particles);
  pf_.setNumPoints(num_control_points);
}

void RoadLine::pfInit()
{
  if (!pf_.initialized())
  {
    float increment = (TOPVIEW_MAX_X - TOPVIEW_MIN_X) / (pf_num_points_ - 1);
    std::vector<cv::Point2f> init_points;
    for (int i = 0; i < pf_num_points_; ++i)
    {
      cv::Point2f p;
      p.x = i * increment + TOPVIEW_MIN_X;
      p.y = getPolyY(coeff_, p.x);
      init_points.push_back(p);
    }
    pf_.init(init_points, pf_std_);
  }
}

void RoadLine::aprox()
{
  polyfit(points_, degree_, coeff_);
}

int RoadLine::pointsSize()
{
  if (points_.empty())
    return 0;

  return points_.size() - 1;
}

void RoadLine::calcParams()
{
  if (!exist_)
  {
    is_short_ = true;
    length_ = 0;
    return;
  }

  length_ = cv::arcLength(points_, false);
  if (length_ > length_not_short_)
    is_short_ = false;
  else
    is_short_ = true;
}