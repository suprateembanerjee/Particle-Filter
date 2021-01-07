/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang

 * Modified on: Jan 3, 2021
 * Author: Suprateem Banerjee
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <limits>
#include <cassert>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::default_random_engine;
using std::discrete_distribution;
using std::numeric_limits;
using std::stringstream;
using std::ostream_iterator;

void ParticleFilter::init(double x, double y, double theta, double std[]) 
{
  // Initialize all particles to first position and all weights to 1. 
  // Add random Gaussian noise to each particle.

  num_particles = 100;  // The number of particles

  weights = vector<double>(num_particles, 1.0);

  default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  particles = vector<Particle>(num_particles);
  for(int i=0;i<num_particles;i++)
  {
    particles[i].id = i;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].weight = weights[i];
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) 
{
  // Add measurements to each particle and add random Gaussian noise.

  default_random_engine gen;
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);

  double op1 = yaw_rate * delta_t;
  double op2 = velocity / yaw_rate;

  for(int i=0; i<particles.size(); i++)
  {
    if(fabs(yaw_rate) < 0.000001)
    {
      particles[i].x += velocity*delta_t*cos(particles[i].theta);
      particles[i].y += velocity*delta_t*sin(particles[i].theta);
    }
    else
    {
      particles[i].x += op2 * (sin(particles[i].theta + op1) - sin(particles[i].theta));
      particles[i].y += op2 * (cos(particles[i].theta) - cos(particles[i].theta + op1));
      particles[i].theta += op1;
    }

    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations)
{
   // Find the predicted measurement that is closest to each observed measurement.
   //  Assign the observed measurement to this particular landmark.

  double min_dist;
  for(int i=0;i<observations.size();i++)
  {
    min_dist = numeric_limits<double>::max();
    observations[i].id = -1;

    for(int j=0; j<predicted.size();j++)
    {
      double distance = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
      if(distance < min_dist)
      {
        min_dist = distance;
        observations[i].id = predicted[j].id;
      }
    }
    assert(observations[i].id != -1);
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) 
{
  //Update the weights of each particle using a mult-variate Gaussian distribution.

  for(int i=0;i<particles.size();i++)
  {
    Particle particle = particles[i]; 
    vector<LandmarkObs> transformed_obs(observations.size());

    for(int j=0; j<observations.size();j++)
    {
      transformed_obs[j].x = particle.x + cos(particle.theta)*observations[j].x - sin(particle.theta)*observations[j].y;
      transformed_obs[j].y = particle.y + sin(particle.theta)*observations[j].x + cos(particle.theta)*observations[j].y;
      transformed_obs[j].id = -1;
    }

    vector<LandmarkObs> landmarks;
    for (int j=0;j<map_landmarks.landmark_list.size();j++)
    {
      Map::single_landmark_s landmark = map_landmarks.landmark_list[j];

      if (dist(particle.x, particle.y, landmark.x_f, landmark.y_f) <= sensor_range)
      {
        LandmarkObs lm = LandmarkObs();
        lm.id = landmark.id_i;
        lm.x = landmark.x_f;
        lm.y = landmark.y_f;
        landmarks.push_back(lm);
      }
    }

    assert(!landmarks.empty());

    dataAssociation(landmarks, transformed_obs);

    vector <double> importance(transformed_obs.size());

    particles[i].weight = 1.0;

    for(int j=0;j<observations.size();j++)
    {
      LandmarkObs transformed_ob = transformed_obs[j];
      LandmarkObs nearest_lm = LandmarkObs();
      nearest_lm.id = -1;
      nearest_lm.x = map_landmarks.landmark_list[transformed_ob.id - 1].x_f;
      nearest_lm.y = map_landmarks.landmark_list[transformed_ob.id - 1].y_f;

      importance[j] = (1 / (2 * M_PI * std_landmark[0] * std_landmark[1])) * exp(-(pow(transformed_ob.x - nearest_lm.x, 2.0) / (2 * pow(std_landmark[0], 2.0)) + pow(transformed_ob.y - nearest_lm.y, 2.0) / (2 * pow(std_landmark[1], 2.0))));
    
      particles[i].weight *= importance[j];
    }

    weights[i] = particles[i].weight;

  }
}

void ParticleFilter::resample() 
{
  // Resample particles with replacement with probability proportional to their weight. 

  default_random_engine gen;
  discrete_distribution<size_t> dist_index(weights.begin(), weights.end());

  vector <Particle> resampled(particles.size());

  for(int i=0; i<particles.size(); i++)
    resampled[i] = particles[dist_index(gen)];

  particles = resampled;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) 
{
  // particle: the particle to which assign each listed association, and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) 
{
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) 
{
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}