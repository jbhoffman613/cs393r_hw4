//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"


#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using std::numeric_limits;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;
using math_util::DegToRad;
using math_util::RadToDeg;
using math_util::AngleDiff;
using math_util::AngleMod;

DEFINE_double(num_particles, 50, "Number of particles");
DEFINE_int32(init_mode, 0, "0 to use location data from set pos, 1 to figure it out");
DEFINE_double(num_particles_init, 2000, "Number of particles");

DEFINE_double(sd_x_from_dist, .2, "Std Dev of local x error from translation");
DEFINE_double(sd_y_from_dist, .05, "Std Dev of local y error from translation");
DEFINE_double(sd_ang_from_dist, 10.0, "Std Dev of angle error in degrees from translation");
DEFINE_double(sd_x_from_rot, 0., "Std Dev of local x error from rotation");
DEFINE_double(sd_y_from_rot, 0., "Std Dev of local y error from rotation");
DEFINE_double(sd_ang_from_rot, 1.0, "Std Dev of angle error in degrees from rotation");

DEFINE_double(gamma, 0.5, "Gamma: LIDAR correlation coefficient");
DEFINE_double(sd_laser, 0.15, "Std Dev");

DEFINE_double(robust_min_sd, 1., "Num std dev for robust cutoff min");
DEFINE_double(robust_max_sd, 3., "Num std dev for robust cutoff max");

DEFINE_int32(loc_algo, 1, "Localization algorithm (0 = mode, 1 = mean)");
DEFINE_int32(robust, 1, "Use robust observation");

DEFINE_double(update_distance, .15, "Distance in meters that robot should move before calling update");
DEFINE_double(update_angle, 1., "Angle in degrees that robot should move before calling update");

DEFINE_int32(resample_algo, 1, "The resample algorithm to use, 0 for default, 1 for low variance");

DEFINE_int32(num_lasers, 100, "The number of lasers to use");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false), 
    odom_update_initialized_(false),
    resampled_last_update_(false) {
      particles_.resize(FLAGS_num_particles);
    }

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  scan.resize(num_ranges);

  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.
  const Eigen::Rotation2Df rotation(angle);
  const Vector2f kLaserLoc(0.2, 0); // Relative to car
  const Vector2f kLaserLoc_m = loc + rotation * kLaserLoc; // relative to map

  vector<line2f> rays(num_ranges); // Relative to map
  float ray_angle = angle_min; // Relative to car
  const float ray_increment = (angle_max - angle_min)/(num_ranges-1);

  // Populate rays with the lines for each lidar measurement
  for (int i = 0; i < num_ranges; i++) {
    // TODO: Check math, especially the sign of angle
    // TODO: Clean up

    // Relative to the car
    const Vector2f ray_rotation(cos(ray_angle), sin(ray_angle));
    Vector2f start = range_min * ray_rotation + kLaserLoc;
    Vector2f end = range_max * ray_rotation + kLaserLoc;

    // Relative to the map
    start = loc + rotation * start;
    end = loc + rotation * end;

    rays[i] = line2f(start, end);

    ray_angle += ray_increment;
  }
  

  // Find closest intersection point for each ray
  for (size_t i = 0; i < rays.size(); i++) {
    const line2f ray_line = rays[i];

    Vector2f closest_point;
    double closest_point_dist = numeric_limits<float>::max();

    // Compare each ray to each line in the map
    for (size_t j = 0; j < map_.lines.size(); j++) {
      const line2f map_line = map_.lines[j];

      Vector2f intersection_point; 
      const bool intersects = ray_line.Intersection(map_line, &intersection_point);

      // rel to map -> rel to lidar
      intersection_point = intersection_point - kLaserLoc_m;
      if (intersects && intersection_point.norm() < closest_point_dist) {
        closest_point = intersection_point;
        closest_point_dist = intersection_point.norm();
      }
    }
    scan[i] = closest_point;
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.

  const int num_ranges = ranges.size();

  double max_log_weight = numeric_limits<double>::lowest();

  const float robust_min_dist = -FLAGS_robust_min_sd * FLAGS_sd_laser;
  const float robust_max_dist = FLAGS_robust_max_sd * FLAGS_sd_laser;

  // Determine log weights
  for (auto& particle : particles_) {
    vector<Vector2f> pred_scan;

    GetPredictedPointCloud(particle.loc, particle.angle, num_ranges, range_min, range_max, angle_min, angle_max, &pred_scan);
    
    // Convert GetPredictedPointCloud to ranges
    vector<double> pred_ranges(pred_scan.size());
    for (uint i = 0; i < pred_scan.size(); i++) {
      pred_ranges[i] = pred_scan[i].norm();
    }

    // TODO: robust weighting
    // Compare the pred_ranges to ranges
    double sum = 0.;
    uint num_used_ranges = 0;

    assert(pred_ranges.size() == ranges.size());

    for (uint i = 0; i < pred_ranges.size(); i++) {
      // Clamp the ranges to be valid
      float pred_range = std::max(std::min((float)pred_ranges[i], range_max), range_min);
      float scan_range = std::max(std::min((float)ranges[i], range_max), range_min);        

      float dist = scan_range - pred_range;
      if (FLAGS_robust) {
        // Clamp distance
        dist = std::min(std::max(dist, robust_min_dist), robust_max_dist);
      }
      sum += pow(dist / FLAGS_sd_laser, 2.);
      num_used_ranges++;
    }

    // Note: in inconsistent state 
    particle.weight = -FLAGS_gamma * sum;
    
    // Find max particle log weight
    if (particle.weight > max_log_weight) {
      max_log_weight = particle.weight;
    }
  }

  // Convert log weights to non-absolute weights
  double weight_sum = 0.; // Only used if FLAGS_loc_algo != 0
  for (auto& particle : particles_) {
    particle.weight -= max_log_weight;
    particle.weight = exp(particle.weight);
    weight_sum += particle.weight;
  }

  // Normalize weights
  if (FLAGS_loc_algo) {
    for (auto& particle : particles_) {
      particle.weight /= weight_sum;
      std::cout << particle.weight << " ";
    }
  }

  std::cout << "\n\n";
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles;
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:

  vector<Particle> new_particles;
  // (1) prefix sum the weights: ps = [0, w_1, ..., sum (0<=i<N) v_i]
  // (2) sample rand (r) from 0, ps[-1] 
  // (3) BS ps for i where ps[i] > r and ps[i-1] < r
  // (4) goto (2) [alt: low-variance sampling]
  

  // Compute prefix sum
  vector<double> prefix_sum(particles_.size() + 1);
  prefix_sum[0] = 0.;
  for (uint i = 1; i < prefix_sum.size(); i++){
    prefix_sum[i] = prefix_sum[i-1] + particles_[i-1].weight;
  }

  uint num_new_particles = particles_.size();

  if (FLAGS_init_mode == 1) {
    num_new_particles = FLAGS_num_particles;
  }
  
  if (!FLAGS_resample_algo) {
    // Regular Resampling
    for (uint i = 0; i < num_new_particles; i++) {
      double rand = rng_.UniformRandom(0., prefix_sum[prefix_sum.size() - 1]);

      // Find index of prefix_sum where ps[j-1] < r && ps[j] > r
      // TODO: Binary Search?
      for (uint j = 1; j < prefix_sum.size(); j++) {
        if (prefix_sum[j] >= rand) {
          new_particles.push_back(Particle{Vector2f(particles_[j-1].loc), particles_[j-1].angle, 1./num_new_particles});
          break;
        }
      }
    }
  } else {
    // Low-variance Resampling
    double sum_weights = prefix_sum[prefix_sum.size() -1];
    double sample_point = rng_.UniformRandom(0, sum_weights);

    double delta = sum_weights/num_new_particles;
    
    for (uint i = 0; i < num_new_particles; i++) {

      // Find index of prefix_sum where ps[j-1] < r && ps[j] > r
      // TODO: Binary Search?
      for (uint j = 1; j < prefix_sum.size(); j++) {
        if (prefix_sum[j] >= sample_point) {
          new_particles.push_back(Particle{Vector2f(particles_[j-1].loc), particles_[j-1].angle, 1./num_new_particles});
          break;
        }
      }
      sample_point = fmod(sample_point + delta, sum_weights);
    }
  }

  // assert(new_particles.size() == particles_.size());
  
  // Update particles
  particles_ = new_particles;

}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  uint num_ranges = ranges.size();
  vector<float> new_ranges;
  uint range_index = 0;
  for (range_index = 0; range_index < ranges.size(); range_index += ranges.size()/FLAGS_num_lasers) {
    new_ranges.push_back(ranges[range_index]);
  }

  range_index -= ranges.size()/FLAGS_num_lasers;
  
  const float ray_increment = (angle_max - angle_min)/(num_ranges - 1);
  angle_max = angle_min + ray_increment*range_index;

  if (odom_update_initialized_ || (prev_update_odom_loc_ - prev_odom_loc_).norm() >= FLAGS_update_distance || abs(prev_update_odom_angle_ - prev_odom_angle_) >= DegToRad(FLAGS_update_angle)) {
    // Update only when robot has changed by a threshold
    Update(new_ranges, range_min, range_max, angle_min, angle_max, nullptr);
    
    prev_update_odom_loc_ = prev_odom_loc_;
    prev_update_odom_angle_ = prev_odom_angle_;
    odom_update_initialized_ = true;
    
    if (!resampled_last_update_) {
      // Resample every other update
      Resample();
    }
    resampled_last_update_ = !resampled_last_update_;
  }
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:

  if (!odom_initialized_){
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
  }

  Vector2f delt_loc = odom_loc - prev_odom_loc_;
  float delt_angle = AngleDiff(odom_angle, prev_odom_angle_);

  Eigen::Rotation2Df r1(-prev_odom_angle_); // rotate odom -> local_prev

  for (auto& particle : particles_) {
    Eigen::Rotation2Df r2(particle.angle); // rotate local_prev -> map
    float sd_x = FLAGS_sd_x_from_dist * delt_loc.norm() + FLAGS_sd_x_from_rot * RadToDeg(abs(delt_angle));
    float sd_y = FLAGS_sd_y_from_dist * delt_loc.norm() + FLAGS_sd_y_from_rot * RadToDeg(abs(delt_angle));
    float sd_ang = DegToRad(FLAGS_sd_ang_from_dist * delt_loc.norm() + FLAGS_sd_ang_from_rot * RadToDeg(abs(delt_angle)));
    particle.loc += r2*(r1*delt_loc + Vector2f(rng_.Gaussian(0., sd_x), rng_.Gaussian(0., sd_y)));
    particle.angle = AngleMod(particle.angle + delt_angle + rng_.Gaussian(0., sd_ang));
  }

  prev_odom_angle_ = odom_angle;
  prev_odom_loc_ = odom_loc;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);


  if (FLAGS_init_mode == 1) {
    particles_.resize(FLAGS_num_particles_init);
    for (auto& particle : particles_) {
      Vector2f loc_offset = Vector2f(rng_.Gaussian(0, 1), rng_.Gaussian(0, 1));
      float ang_offset = rng_.Gaussian(0, M_PI/4.);
      particle.loc = loc + loc_offset;
      particle.angle = angle + ang_offset;
      particle.weight = 1./(double)FLAGS_num_particles_init;
    }
  } else {
    particles_.resize(FLAGS_num_particles);
    for (auto& particle : particles_) {
      Vector2f loc_offset = Vector2f(rng_.UniformRandom(-0.2, 0.2), rng_.UniformRandom(-0.005, 0.005));
      float ang_offset = rng_.UniformRandom(-M_PI/2., M_PI/2.);
      particle.loc = loc + loc_offset;
      particle.angle = angle + ang_offset;
      particle.weight = 1./(double)FLAGS_num_particles;
    }
  }
  odom_initialized_ = false;
  odom_update_initialized_ = false;
  resampled_last_update_ = false;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  
  if (!FLAGS_loc_algo) {
    // MODE: ie return particle with highest weight
    Particle mode_particle{Vector2f(0., 0.), 0., 0.};
    for (auto& particle : particles_) {
      if (particle.weight > mode_particle.weight) {
        mode_particle = particle;
      }
      std::cout << particle.weight << " ";
    }
    std::cout << "\n\n";
    loc = mode_particle.loc;
    angle = mode_particle.angle;
  } else {
    // MEAN: ie return weighted average of loc and angle
    // NOTE: Requires weights to be normalized
    loc = Vector2f(0., 0.);
    Vector2f angle_iq = Vector2f(0., 0.);

    for (auto& particle : particles_) {
      loc += particle.loc * particle.weight;
      angle_iq = angle_iq + Vector2f(cos(particle.angle), sin(particle.angle)) * particle.weight;
    }

    angle = atan2(angle_iq[1], angle_iq[0]);
  }
}

}  // namespace particle_filter
