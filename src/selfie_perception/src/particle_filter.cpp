/**
 * Copyright (c) 2019 Koło Naukowe Robotyków
 * This code is licensed under BSD license (see LICENSE for details)
 **/

#include <selfie_perception/particle_filter.h>

void ParticleFilter::init(std::vector<cv::Point2f> points, double std)
{
  if (num_control_points_ != points.size())
  {
    std::cout << "Invalid control points number" << std::endl;
    return;
  }
  weights_.resize(num_particles_);
  particles_.resize(num_particles_);

  std::vector<std::normal_distribution<float>> dist;
  std::default_random_engine gen;

  for (int i = 0; i < num_control_points_; ++i)
  {
    std::normal_distribution<float> dist_i(points[i].y, std);
    dist.push_back(dist_i);
  }

  for (int i = 0; i < num_particles_; ++i)
  {
    Particle p;
    p.id = i;
    for (int j = 0; j < num_control_points_; ++j)
    {
      cv::Point2f pt;
      pt.y = dist[j](gen);
      pt.x = points[j].x;
      p.points.push_back(pt);
    }
    p.weight = 1;

    if (!polyfitt(p.points, poly_degree_, p.coeff))
    {
      p.coeff.push_back(0.0);
    }

    particles_[i] = p;
    weights_[i] = p.weight;
  }
  is_initialized_ = true;
}

void ParticleFilter::prediction(double std)
{
  std::random_device r;
  std::default_random_engine gen(r());

  for (int i = 0; i < num_particles_; ++i)
  {
    for(int j = 0; j < num_control_points_; ++j)
    {
      std::normal_distribution<float> dist(particles_[i].points[j].y, std);
      particles_[i].points[j].y = dist(gen);
    }
    if (!polyfitt(particles_[i].points, poly_degree_, particles_[i].coeff))
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
      distance_sum += std::fabs(p_obs[j].y - getY(particles_[i].coeff, p_obs[j].x));
    }
    //std::cout << distance_sum << std::endl;
    particles_[i].weight = 1 / (1 + exp(9 * distance_sum));
    weights_sum += distance_sum;
  }

  for (int i = 0; i < num_particles_; ++i)
  {
    particles_[i].weight /= weights_sum;
    std::cout << particles_[i].weight << std::endl;
    weights_[i] = particles_[i].weight;
  }
}

void ParticleFilter::resample()
{
  std::default_random_engine gen;
  std::discrete_distribution<int> dist(weights_.begin(), weights_.end());
  std::vector<Particle> resampled_particles;

  for (int i = 0; i < num_particles_; ++i)
  {
    resampled_particles.push_back(particles_[dist(gen)]);
  }
  particles_ = resampled_particles;
}

std::vector<float> ParticleFilter::getCoeff(int particle_id)
{
  return particles_[particle_id].coeff;
}

std::vector<float> ParticleFilter::getBestCoeff()
{
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
  return particles_[max_index].coeff;
}

void ParticleFilter::reset()
{
  weights_.clear();
  particles_.clear();
  is_initialized_ = false;
}