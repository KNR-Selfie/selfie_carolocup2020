#include <selfie_perception/road_line.h>

RoadLine::RoadLine()
{
  // init straight line with 0.2 offset
  coeff_.clear();
  coeff_.push_back(0.2);
  coeff_.push_back(0);
  coeff_.push_back(0);
}

void RoadLine::pfSetup(int num_particles, int num_control_points, float std_min, float std_max)
{
  pf_num_particles_ = num_particles;
  pf_num_points_ = num_control_points;
  pf_std_min_ = std_min;
  pf_std_max_ = std_max;

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
    pf_.init(init_points);
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

  pf_.prediction(pf_std_min_, pf_std_max_);
  
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
  if(!polyfit(points_, degree_, coeff_))
  {
    coeff_.clear();
    coeff_.push_back(0.2);
    coeff_.push_back(0);
    coeff_.push_back(0);
  }
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

void RoadLine::addBottomPoint(bool force)
{
  if (!exist_)
    return;

  if (force || points_[0].x < ((TOPVIEW_MIN_X + TOPVIEW_MAX_X) / 3))
  {
    cv::Point2f p;
    p.x = TOPVIEW_MIN_X;
    if (force)
    {
      p.y = points_[pointsSize()].y;
    }
    else
    {
      p.y = points_[0].y;
    }
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
  pf_.reset();

  // init straight line with 0.2 offset
  coeff_.clear();
  coeff_.push_back(0.2);
  coeff_.push_back(0);
  coeff_.push_back(0);
}

void RoadLine::reduceTopPoints(float ratio)
{
  if (points_.empty())
    return;

  int begin = points_.size() * (1 - ratio);
  points_.erase(points_.begin() + begin, points_.end());
}

cv::Point2f RoadLine::getPointNextToBottom(float min_dist_to_bottom)
{
  for(int i = 1; i < pointsSize(); ++i)
  {
    if (points_[i].x - points_[0].x > min_dist_to_bottom)
      return points_[i];
  }
  return points_[pointsSize() - 1];
}

void RoadLine::reducePointsToStraight(int check_to_index)
{
  if(pointsSize() == 0)
    return;

  points_.erase(points_.begin() + check_to_index, points_.end());

  // generate for density
  for (int i = 0; i < pointsSize(); ++i)
  {
    float distance = getDistance(points_[i], points_[i + 1]);
    if (distance > 1 / 20)
    {
      int add = distance * 20;
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

  float max = 0;
  int index_max = -1;

  float A = getA(points_[0], points_[pointsSize() - 1]);
  float C = points_[0].y - (A * points_[0].x);
  float sqrtf_m = sqrtf(A * A + 1);

  for(int i = 2; i < pointsSize(); ++i)
  {
    float distance = std::abs(A * points_[i].x - points_[i].y + C) / sqrtf_m;
    if(distance > max)
    {
      max = distance;
      index_max = i;
    }
  }
  if (max > 0.01 && index_max > 4)
  {
    points_.erase(points_.begin() + index_max, points_.end());
  }
}

float RoadLine::getA(cv::Point2f p1, cv::Point2f p2)
{
  return (p2.y - p1.y) / (p2.x - p1.x);
}

float RoadLine::getMaxDiffonX()
{
  if(pointsSize() == 0)
    return 0;

  float dist = 0;
  for(int i = 1; i < pointsSize(); ++i)
  {
    float dist_now = points_[i].x - points_[i - 1].x;
    if (dist_now > dist)
    {
      dist = dist_now;
    }
  }
  return dist;
}

int RoadLine::getIndexOnMerge()
{
  for(int i = 1; i < pointsSize(); ++i)
  {
    float dist_now = points_[i].x - points_[i - 1].x;
    if (dist_now > 0.3)
    {
      return i - 1;
    }
  }
  return pointsSize();
}
