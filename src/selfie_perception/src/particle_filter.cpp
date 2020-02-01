/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <selfie_perception/particle_filter.h>

void ParticleFilter::init(std::vector<cv::Point2f> points)
{
  if (num_control_points_ != points.size())
  {
    std::cout << "Invalid control points number" << std::endl;
    return;
  }
  weights_.resize(num_particles_);
  particles_.resize(num_particles_);

  for (int i = 0; i < num_particles_; ++i)
  {
    Particle p;
    for (int j = 0; j < num_control_points_; ++j)
    {
      cv::Point2f pt;
      pt.y = points[j].y;
      pt.x = points[j].x;
      p.points.push_back(pt);
      p.poly_degree = poly_degree_;
    }
    p.weight = 1;

    if (!polyfit(p.points, p.poly_degree, p.coeff))
    {
      p.coeff.push_back(0.0);
    }

    particles_[i] = p;
    weights_[i] = p.weight;
  }
  is_initialized_ = true;
}

void ParticleFilter::prediction(float std_min, float std_max)
{
  std::random_device r;
  std::default_random_engine gen(r());

  for (int i = 0; i < num_particles_; ++i)
  {
    for(int j = 0; j < num_control_points_; ++j)
    {
      //float deriative = 2 * particles_[i].coeff[2] * particles_[i].points[j].x + particles_[i].coeff[1];
      //float angle = atan(deriative);

      //std::normal_distribution<float> dist_x(0, std * sin(angle));
      //std::normal_distribution<float> dist_y(0, std * cos(angle));
      float std = (std_max - std_min) / (TOPVIEW_MAX_X - TOPVIEW_MIN_X) * (particles_[i].points[j].x - TOPVIEW_MIN_X) + std_min;

      std::normal_distribution<float> dist_y(0, std);
      std::normal_distribution<float> dist_x(0, std);

      particles_[i].points[j].x += dist_x(gen);
      if (particles_[i].points[j].x > TOPVIEW_MAX_X)
      {
        particles_[i].points[j].x = TOPVIEW_MAX_X;
      }
      else if (particles_[i].points[j].x < TOPVIEW_MIN_X)
      {
        particles_[i].points[j].x = TOPVIEW_MIN_X;
      }

      particles_[i].points[j].y += dist_y(gen);
    }
    if (!polyfit(particles_[i].points, particles_[i].poly_degree, particles_[i].coeff))
    {
      particles_[i].coeff.clear();
      particles_[i].coeff.push_back(0.0);
    }
  }
}

void ParticleFilter::updateWeights(std::vector<cv::Point2f> &p_obs)
{
  float weights_sum = 0;
  for (int i = 0; i < num_particles_; ++i)
  {
    float distance_sum = 0;
    for (int j = 0; j < p_obs.size(); ++j)
    {
      distance_sum += findMinPointToParabola(p_obs[j], particles_[i].coeff);
    }
    particles_[i].weight = 1 / (1 + exp(9 * distance_sum));
    weights_sum += particles_[i].weight;
  }

  for (int i = 0; i < num_particles_; ++i)
  {
    particles_[i].weight /= weights_sum;
    weights_[i] = particles_[i].weight;
  }
}

void ParticleFilter::resample()
{
  std::vector<Particle> resampled_particles;
  std::random_device rd;
  std::default_random_engine gen(rd());
  float invM = 1 / float(num_particles_);
  std::uniform_real_distribution<> dis(0, invM);
  float c = weights_[0];
  float r = dis(gen);
  int i = 0;
  for (int m = 0; m < num_particles_; ++m)
  {
    float U = r + m * invM;
    while (U > c)
    {
      ++i;
      c += weights_[i];
    }
    resampled_particles.push_back(particles_[i]);
  }
  particles_ = resampled_particles;
/*
  int max_index = 0;
  float max = 0;
  for (int i = 0; i < particles_.size(); ++i)
  {
    if(particles_[i].weight > max)
    {
      max = particles_[i].weight;
      max_index = i;
    }
  }
  best_particle_ = particles_[max_index];
*/
  calculateBest();
}

std::vector<float> ParticleFilter::getCoeff(int particle_id)
{
  if (particle_id < num_particles_)
  {
    return particles_[particle_id].coeff;
  }
  else
  {
    std::vector<float> empty;
    empty.push_back(0.0);
    return empty;
  }
}

std::vector<float> ParticleFilter::getBestCoeff()
{
  return best_particle_.coeff;
}

int ParticleFilter::getBestDegree()
{
  return best_particle_.poly_degree;
}

std::vector<cv::Point2f> ParticleFilter::getControlPoints(int particle_id)
{
  if (particle_id < num_particles_)
  {
    return particles_[particle_id].points;
  }
  else
  {
    std::vector<cv::Point2f> empty;
    cv::Point2f p;
    p.x = 0;
    p.y = 0;
    empty.push_back(p);
    return empty;
  }
}

float ParticleFilter::findMinPointToParabola(cv::Point2f p, std::vector<float> coeff)
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
  return std::fabs(min);
}

float ParticleFilter::getDistance(cv::Point2f p1, cv::Point2f p2)
{
  float dx = (p1.x - p2.x);
  float dy = (p1.y - p2.y);
  return sqrtf(dx * dx + dy * dy);
}

void ParticleFilter::calculateBest()
{
  std::vector<cv::Point2f> points_tab;
  points_tab.resize(num_control_points_);
  for (int i = 0; i < particles_.size(); ++i)
  {
    for (int j = 0; j < num_control_points_; ++j)
    {
      points_tab[j].x += particles_[i].points[j].x;
      points_tab[j].y += particles_[i].points[j].y;
    }
  }
  for (int j = 0; j < num_control_points_; ++j)
  {
    points_tab[j].x /= num_particles_;
    points_tab[j].y /= num_particles_;
  }
  polyfit(points_tab, poly_degree_, best_particle_.coeff);
  best_particle_.poly_degree = poly_degree_;
}

void ParticleFilter::reset()
{
  weights_.clear();
  particles_.clear();
  is_initialized_ = false;
}