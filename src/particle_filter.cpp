#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /***** 1. STEP: INITIALIZATION *****
  * Initialize all particles (x, y, theta) based on GPS input (+ Gaussian noise), set all weights to 1.0.
  */
  
  // Set the number of particles
  num_particles = 100;
  
  // Initialize random engine
  std::default_random_engine gen;
  
  // Set standard deviations for x, y, and theta
  double std_x, std_y, std_theta;
  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];
  
  // Create normal (Gaussian) distributions with mean on GPS input
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  
  // Add random Gaussian noise to each particle, "gen" is the random engine initialized earlier
  for (int i = 0; i < num_particles; i++) {    
    Particle ith_particle;
    ith_particle.id = i;
    ith_particle.x = dist_x(gen);
    ith_particle.y = dist_y(gen);
    ith_particle.theta = dist_theta(gen);
    ith_particle.weight = 1.0;
    particles.push_back(ith_particle);
    weights.push_back(ith_particle.weight);
  }
  // Set filter initialized true
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /***** 2. STEP: PREDICTION *****
  * Add the control input x, y and the yaw angle to all particles (+ Gaussian noise) 
  */
  
  // Initialize random engine
  std::default_random_engine gen;
  
  // Set standard deviations for x, y, and theta
  double std_pos_x, std_pos_y, std_pos_theta;
  std_pos_x = std_pos[0];
  std_pos_y = std_pos[1];
  std_pos_theta = std_pos[2];
  
  // Create normal (Gaussian) distributions with mean zero
  normal_distribution<double> dist_x(0, std_pos_x);
  normal_distribution<double> dist_y(0, std_pos_y);
  normal_distribution<double> dist_theta(0, std_pos_theta);
  
  // Add control input to each particle
  for (int i = 0; i < num_particles; i++) {
    
    double theta = particles[i].theta;
    
    // check if yaw rate is zero and avoid division by zero error
    if (fabs(yaw_rate) < 0.0001){
      particles[i].x += velocity * cos(theta)*delta_t;
      particles[i].y += velocity * sin(theta)*delta_t ;
    }
    else{
      particles[i].x += velocity/yaw_rate * (sin(theta + yaw_rate*delta_t) - sin(theta));
      particles[i].y += velocity/yaw_rate * (cos(theta) - cos(theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }
    
    // Add Gaussian noise
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  
  /***** 3. STEP: DATA ASSOCIATION WITH NEAREST NEIGHBOR TECHNIQUE *****
  *Find the predicted measurement that is closest to each observed measurement and
  assign the observed measurement to this particular landmark.
  */
  
  for (unsigned int i = 0; i < observations.size(); i++) {
    int neighbor_landmark_id;
    // Initialize the temporary distance variable with 1000m, it will later hold the distances to the landmarks and is used to compare against
    double temp_dist = 1000.; 
    double observ_x = observations[i].x;
    double observ_y = observations[i].y;

    for (unsigned int j = 0; j < predicted.size(); j++) {
      double predict_x = predicted[j].x;
      double predict_y = predicted[j].y;
      int predict_id = predicted[j].id;
      // Use dist()-function from helper.h, it computes and returns the Euclidean distance between two 2D points
      double distance = dist(observ_x, observ_y, predict_x, predict_y);
      
      if (distance < temp_dist) {
        temp_dist = distance;
        neighbor_landmark_id = predict_id;
      }
    }
    observations[i].id = neighbor_landmark_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {

  /***** 4. STEP: UPDATE THE PARTICLES WEIGHTS *****
  * 1. Transform observations from vehicle's coordinates to map coordinates.
  * 2. Filter on landmarks that are in range of the LiDAR.
  * 3. Associate observations with predicted landmarks
  * 4. Update the weights of each particle using a multvariate Gaussian distribution
  * 5. Normalize the weights of all particles for the resampling step
  */
  

  /*This variable is used for normalizing weights of all particles and bring them in the range
    of [0, 1]*/
  double weight_normalizer = 0.0;

  for (int i = 0; i < num_particles; i++) {
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;

    // 1. Transform observations from vehicle's coordinates to map coordinates.
    vector<LandmarkObs> transformed_observations;

    for (unsigned int j = 0; j < observations.size(); j++) {
      LandmarkObs trans_obs;
      trans_obs.id = j;
      trans_obs.x = x + (cos(theta) * observations[j].x) - (sin(theta) * observations[j].y);
      trans_obs.y = y + (sin(theta) * observations[j].x) + (cos(theta) * observations[j].y);
      transformed_observations.push_back(trans_obs);
    }

    // 2. Filter on landmarks that are in range of the LiDAR
    vector<LandmarkObs> predicted_landmarks;
    
    for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); k++) {
      Map::single_landmark_s temp_landmark;
      temp_landmark = map_landmarks.landmark_list[k];
      
      if ((fabs((x - temp_landmark.x_f)) <= sensor_range) && (fabs((y - temp_landmark.y_f)) <= sensor_range)) {
        predicted_landmarks.push_back(LandmarkObs {temp_landmark.id_i,
                                                   temp_landmark.x_f,
                                                   temp_landmark.y_f});
      }
    }

    
    // 3. Associate observations with predicted landmarks
    dataAssociation(predicted_landmarks, transformed_observations);

    // 4. Update the weights of each particle using a multvariate Gaussian distribution
    particles[i].weight = 1.0;

    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];
    double sigma_x_2 = pow(sigma_x, 2);
    double sigma_y_2 = pow(sigma_y, 2);
    double normalizer = (1.0/(2.0 * M_PI * sigma_x * sigma_y));
    
    for (unsigned int l = 0; l < transformed_observations.size(); l++) {
      double trans_obs_x = transformed_observations[l].x;
      double trans_obs_y = transformed_observations[l].y;
      double trans_obs_id = transformed_observations[l].id;
      double multi_var_prob;

      for (unsigned int m = 0; m < predicted_landmarks.size(); m++) {
        double pred_landmark_x = predicted_landmarks[m].x;
        double pred_landmark_y = predicted_landmarks[m].y;
        double pred_landmark_id = predicted_landmarks[m].id;

        if (trans_obs_id == pred_landmark_id) {
          multi_var_prob = normalizer * exp(-1.0 * ((pow((trans_obs_x - pred_landmark_x), 2)/(2.0 * sigma_x_2)) + (pow((trans_obs_y - pred_landmark_y), 2)/(2.0 * sigma_y_2))));
          particles[i].weight *= multi_var_prob;
        }
      }
    }
    weight_normalizer += particles[i].weight;
  }
  // 5. Normalize the weights of all particles for the resampling step
  for (unsigned int i = 0; i < particles.size(); i++) {
    particles[i].weight /= weight_normalizer;
    weights[i] = particles[i].weight;
  }  

}

void ParticleFilter::resample() {

  /***** 5. STEP: RESAMPLE PARTICLES WITH REPLACEMENT WITH PROBABILITY PROPORTIONAL TO THEIR WEIGHT *****
   * With use of Resampling Wheel technique
   */
  
  vector<Particle> resampled_particles;
  
  // Initialize random engine
  std::default_random_engine gen;
  
  // Generate random particle index
  std::uniform_int_distribution<int> particle_index(0, num_particles - 1);
  
  int current_index = particle_index(gen);
  
  double beta = 0.0;
  
  double max_weight_2 = 2.0 * *max_element(weights.begin(), weights.end());
  
  // Resampling Wheel
  for (unsigned int i = 0; i < particles.size(); i++) {
    std::uniform_real_distribution<double> random_weight(0.0, max_weight_2);
    beta += random_weight(gen);
    
    while (beta > weights[current_index]) {
      beta -= weights[current_index];
      current_index = (current_index + 1) % num_particles;
    }
    resampled_particles.push_back(particles[current_index]);
  }
  particles = resampled_particles;
  
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}