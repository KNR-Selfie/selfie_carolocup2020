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
  if (!pf_.initialized() && exist_)
  {
    float poly_topview_x = TOPVIEW_MAX_X;
    float p_top_y = getPolyY(coeff_, TOPVIEW_MAX_X);
    if (p_top_y < TOPVIEW_MIN_Y || p_top_y > TOPVIEW_MAX_Y)
    {
      pf_.setPolyDegree_(2);
      for (int i = 1; i < 20; ++i)
      {
        p_top_y = getPolyY(coeff_, TOPVIEW_MAX_X - 0.05 * i);
        if (p_top_y > TOPVIEW_MIN_Y && p_top_y < TOPVIEW_MAX_Y)
        {
          poly_topview_x = TOPVIEW_MAX_X - 0.05 * i;
          break;
        }
      }
    }

    float increment = (poly_topview_x - TOPVIEW_MIN_X) / (pf_num_points_ - 1);
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

bool RoadLine::pfExecute()
{
  if (!exist_)
    return false;
  
  if (!pf_.initialized())
  {
    pf_.reset();
    pfInit();
  }
  else
  {
    pf_.prediction(pf_std_);
  }
  
  pf_.updateWeights(points_);
  pf_.resample();
  coeff_ = pf_.getBestCoeff();
  degree_ = pf_.getBestDegree();

  return true;
}

void RoadLine::pfReset()
{
  pf_.reset();
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
    points_.clear();
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

void RoadLine::addBottomPoint()
{
  if (!exist_)
    return;

  if (points_[0].x < ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 3))
  {
    cv::Point2f p;
    p.x = TOPVIEW_MIN_X;
    p.y = points_[0].y;
    points_.insert(points_.begin(), p);
  }
}

void RoadLine::generateForDensity()
{
  if (!exist_)
  {
    return;
  }

  if (pointsSize() / length_ > points_density_ * 2)
  {
    return;
  }

  for (int i = 0; i < pointsSize(); ++i)
  {
    float distance = getDistance(points_[i], points_[i + 1]);
    if (distance > 1 / points_density_)
    {
      int add = distance * points_density_;
      cv::Point2f p;
      float x1 = points_[i].x;
      float y1 = points_[i].y;
      float x_dif = (points_[i + 1].x - points_[i].x) / (add + 1);
      float y_dif = (points_[i + 1].y - points_[i].y) / (add + 1);
      for (int j = 0; j < add; ++j)
      {
        p.x = x1 + x_dif * (j + 1);
        p.y = y1 + y_dif * (j + 1);
        points_.insert(points_.begin() + i + 1, p);
        ++i;
      }
    }
  }
}

float RoadLine::getDistance(cv::Point2f p1, cv::Point2f p2)
{
  float dx = (p1.x - p2.x);
  float dy = (p1.y - p2.y);
  return sqrtf(dx * dx + dy * dy);
}

void RoadLine::reset()
{
  exist_ = false;
  degree_ = 2;
  length_ = 0;
  is_short_ = true;
  points_.clear();
  coeff_.clear();
  pf_.reset();
}