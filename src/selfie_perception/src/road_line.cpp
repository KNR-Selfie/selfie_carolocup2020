#include <selfie_perception/road_line.h>

RoadLine::RoadLine()
{
  float empty_coeff = 0;
    coeff_.push_back(empty_coeff);
    coeff_.push_back(empty_coeff);
    coeff_.push_back(empty_coeff);
}

RoadLine::pfSetup(int num_particles, int num_control_points, float std)
{
  pf_num_particles_ = num_particles;
  pf_num_points_ = num_control_points;
  pf_std_ = std;

  pf.setNumParticles(num_particles);
  pf.setNumPoints(num_control_points);
}

RoadLine::pfInit();
{
  if (!pf.initialized())
  {
    float increment = (TOPVIEW_MAX_X - TOPVIEW_MIN_X) / (pf_num_points_ - 1);
    std::vector<cv::Point2f> init_points;
    for (int i = 0; i < pf_num_points_; ++i)
    {
      cv::Point2f p;
      p.x = i * increment + TOPVIEW_MIN_X;
      p.y = getPolyY(center_line_.coeff, p.x);
      init_points.push_back(p);
    }
    pf.init(init_points, pf_std_);
  }
}

RoadLine::aprox()
{
  polyfit(points_, degree_, coeff_);
}