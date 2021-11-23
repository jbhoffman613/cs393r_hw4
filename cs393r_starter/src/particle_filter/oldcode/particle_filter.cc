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
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");
DEFINE_double(num_rays, 50, "Number of LIDAR rays per particle");

// TUNABLE CONSTANTS
const float K1 = 0.3;//0.3;
const float K2 = 0.5;//0.5;
const float K3 = 1.5; //0.75;
const float K4 = 1.5; //1.5;
const float INIT_STDDEV_R = 0.15; // radians
const float INIT_STDDEV_T = 0.15;    // distance in meters
const float LIDAR_RANGE_CAP = 20; // distance in meters
const float GAMMA = 0.7;          // fraction between 1/n and 1
const float SENSOR_STDDEV = 0.15; // distance in meters
const float D_SHORT = 0.35;          // distance in meters
const float D_LONG = 0.01;           // distance in meters

// CONSTANT CONSTANTS (No need to change)
const float LIDAR_OFFSET = 0.19;
const float LARGE_RAY_FLAG = 15023494; // must be larger than 10^7

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

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
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.
  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);

  // for the given particle, we should translate into the lidar frame 
  // and obtain the line segments representing the lidar readings
  

  // find the coordinates of the lidar sensor on the map frame
  // TODO: is this right?
  float lidar_x = loc(0) + (LIDAR_OFFSET * std::cos(angle));
  float lidar_y = loc(1) + (LIDAR_OFFSET * std::sin(angle));
  Vector2f lidar_map_loc(lidar_x, lidar_y); // location of particle's lidar sensor in the map frame


  // generate a set of lidar rays from the particle's lidar point
  vector<line2f> lidar_rays;
  float angle_increment = (angle_max - angle_min) / num_ranges; // angle between lidar rays
  float current_lidar_angle = angle_min + angle;
  for (int i = 0; i < num_ranges; i++) {
    line2f new_line = BuildLineSegment(lidar_map_loc, current_lidar_angle, range_min, range_max);
    lidar_rays.push_back(new_line);
    current_lidar_angle = current_lidar_angle + angle_increment;
  }

  // now go through and find intersections
  Vector2f intersection_point;
  bool intersects;
  float min_distance;
  Vector2f closest_intersection;
  float dist_to_intersection;

  for (size_t i = 0; i < lidar_rays.size(); ++i) {
    const line2f lidar_line = lidar_rays[i];

    min_distance = 1000001;
    closest_intersection = Vector2f(1000000, 1000000); // very large default value shouldn't overlap with real values

    for (size_t j = 0; j < map_.lines.size(); ++j) {
      const line2f map_line = map_.lines[j];

      intersects = map_line.Intersection(lidar_line, &intersection_point);
      // if we find an intersection, determine whether it is closer to the lidar than the current closest point
      if (intersects) {
        // dist_to_intersection = std::sqrt( std::pow(lidar_map_loc(0) - intersection_point(0), 2) + std::pow(lidar_map_loc(1) - intersection_point(1), 2) );
        dist_to_intersection = std::hypot(lidar_map_loc(0) - intersection_point(0), lidar_map_loc(1) - intersection_point(1));

        if (dist_to_intersection < min_distance) {
          min_distance = dist_to_intersection;
          closest_intersection = intersection_point;
        }
      } else if (min_distance > 1000000) {
        // TODO check for the min distance and if it is too large set it yp like neg 1 or something. 
        // We can then ignore it maybe or throw it out. Just make sure nothing is hard coded. 
        // maybe set to intmax or something just to check it? 
        closest_intersection = Vector2f(lidar_line.p1.x(), lidar_line.p1.y());
      }
    }
    if (min_distance > LIDAR_RANGE_CAP) {
      scan[i] = Vector2f(LARGE_RAY_FLAG, LARGE_RAY_FLAG);
    } else {
      scan[i] = closest_intersection;
    }
  }
}

line2f ParticleFilter::BuildLineSegment(Vector2f lidar_loc, 
                                        float angle, 
                                        float range_min, 
                                        float range_max) {
  float x1 = lidar_loc(0) + (range_min * cos(angle));
  float y1 = lidar_loc(1) + (range_min * sin(angle));
  float x2 = lidar_loc(0) + (range_max * cos(angle));
  float y2 = lidar_loc(1) + (range_max * sin(angle));

  return line2f(x1, y1, x2, y2);
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

  float lidar_x = p_ptr->loc(0) + (LIDAR_OFFSET * std::cos(p_ptr->angle));
  float lidar_y = p_ptr->loc(1) + (LIDAR_OFFSET * std::sin(p_ptr->angle));

  vector<Vector2f> scan_ptr;
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, FLAGS_num_rays, range_min, range_max, angle_min, angle_max, &scan_ptr);
  vector<float> lidar_ranges(FLAGS_num_rays);
  vector<float> lidar_angles;
  vector<float> pred_ranges;

  float angle_incr = (angle_max - angle_min) / FLAGS_num_rays;
  for (int i = 0; i < FLAGS_num_rays; i++) {
    lidar_angles.push_back(angle_min + i * angle_incr);
  }
  
  // find lidar ranges for corresponding simulated ranges
  // TODO: fix this
  // float idx_increment_pred = ranges.size() / FLAGS_num_rays;
  // for (int j = 0; j < FLAGS_num_rays; j++) {
  //   size_t range_idx = std::round(j*idx_increment_pred);
  //   if (ranges.size() == range_idx) {
  //     range_idx -= 1;
  //   } 
  //   lidar_ranges.push_back(ranges[range_idx]);
  // }

  angle_incr = (angle_max - angle_min) / ranges.size();
  for (int j = 0; j < FLAGS_num_rays; j++) {
    int closest_idx = 0;
    float min_angle_diff = 7;
    for (size_t i = 0; i < ranges.size(); i++) {
      float curr_angle = angle_incr * i + angle_min;
      float angle_diff = (std::abs(lidar_angles[j] - curr_angle));
      if (angle_diff < min_angle_diff) {
        min_angle_diff = angle_diff;
        closest_idx = i;
      }
    }
    lidar_ranges[j] = ranges[closest_idx];
  }

  // calculate distances between intersection and simulated lidar for predicted point cloud
  for (size_t j = 0; j < scan_ptr.size(); j++) {
    if (scan_ptr[j](0) == LARGE_RAY_FLAG) {
      pred_ranges.push_back(-1);
    } else {
      pred_ranges.push_back(std::hypot(scan_ptr[j](0) - lidar_x, scan_ptr[j](1) - lidar_y));
    }
  }

  // calculate log likelihood
  p_ptr->weight = 0;
  int num_rays_counted = FLAGS_num_rays;
  for (int j = 0; j < FLAGS_num_rays; j++) {
    if (pred_ranges[j] == -1) {
      num_rays_counted--;
      continue;
    }

    if (lidar_ranges[j] > range_max || lidar_ranges[j] < range_min)
      continue;
    else if (lidar_ranges[j] < pred_ranges[j] - D_SHORT)
      p_ptr->weight += -1 * GAMMA * std::pow(D_SHORT, 2) / std::pow(SENSOR_STDDEV, 2); // D_SHORT SHOULD BE BIGGER
    else if (lidar_ranges[j] < pred_ranges[j] + D_LONG)
      p_ptr->weight += -1 * GAMMA * std::pow(D_LONG, 2) / std::pow(SENSOR_STDDEV, 2);
    else
      p_ptr->weight += -1 * GAMMA * std::pow(pred_ranges[j] - lidar_ranges[j], 2) / std::pow(SENSOR_STDDEV, 2);
  }

  std::cout << "num rays dropped: " << FLAGS_num_rays - num_rays_counted << std::endl;
  p_ptr->weight /= num_rays_counted;
}

void ParticleFilter::Resample() {
  if (particles_.size() == 0) {
    return;
  }

  // normalize - MOVED TO ObserveLaser()
  // float sum = 0;
  // for (size_t i = 0; i < FLAGS_num_particles; i++) {
  //   sum += particles_[i].weight;
  // }

  // for (size_t i = 0; i < FLAGS_num_particles; i++) {
  //   particles_[i].weight = particles_[i].weight / sum;
  // }

  // set up weights vector
  vector<float> weights;
  weights.push_back(particles_[0].weight);
  for (size_t i = 1; i < FLAGS_num_particles; i++) {
    weights.push_back(particles_[i].weight + weights[i-1]);
  }

  // pick a seed between 0 and 1/FLAG_num_ranges
  float step_size = 1.0/FLAGS_num_particles;
  float seed = rng_.UniformRandom(0, step_size);
  int weight_index = 0;
  float current_sample;
  vector<struct Particle> new_particles;
  for (size_t i = 0; i < FLAGS_num_particles; i++) {
    current_sample = seed + (i * step_size);
    while (current_sample > weights[weight_index]) {
      weight_index++;
    }
    new_particles.push_back(particles_[weight_index]);
  }

  particles_ = new_particles;
  for (size_t i = 0; i < FLAGS_num_particles; i++) {
    particles_[i].weight = 1.0 / FLAGS_num_particles;
  }
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min, 
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  double max_log_likelihood = -10000000;
  for (size_t i = 0; i < particles_.size(); i++) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &particles_[i]);
    assert(particles_[i].weight <= 0);
    max_log_likelihood = std::max(max_log_likelihood, particles_[i].weight);
  }
  // std::cout << "max log likelihood: " << max_log_likelihood << std::endl;
  for (size_t i = 0; i < particles_.size(); i++) {
    // std::cout << "weight - maxloglike: " << particles_[i].weight - max_log_likelihood << std::endl;
    particles_[i].weight = std::exp(particles_[i].weight - max_log_likelihood);

  }
  // std::cout << std::endl;

    // normalize
  float sum = 0;
  for (size_t i = 0; i < particles_.size(); i++) {
    sum += particles_[i].weight;
  }

  for (size_t i = 0; i < particles_.size(); i++) {
    particles_[i].weight = particles_[i].weight / sum;
    // std::cout << particles_[i].weight << std::endl; 
  }
  // std::cout << std::endl;

  Resample();
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  if (!odom_initialized_) {
    odom_initialized_ = true;
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    return;
  }

  if (particles_.size() == 0) {
    return;
  }

  // compute the noise free location for each particle
  for (unsigned int i = 0; i < particles_.size(); i++) {
    // transform from odom frame to map frame
    particles_[i] = NoiseFreeNextLocation(odom_loc, odom_angle, particles_[i]);
    // sample from gaussians
    particles_[i] = SampleMotionModel(odom_loc, odom_angle, particles_[i]);
  }

  prev_odom_angle_ = odom_angle;
  prev_odom_loc_ = odom_loc;
}

std::tuple<float, float, float> ParticleFilter::ComputeDeltaBaseLink(const Vector2f& odom_loc, const float odom_angle) {
  float change_x = odom_loc(0) - prev_odom_loc_(0);
  float change_y = odom_loc(1) - prev_odom_loc_(1);

  float neg_angle = -1.0 * prev_odom_angle_;

  // compute R(-\theta^{odom}_1)*(T^{odom}_2 - T^{odom}_1) for x and y
  float x_prime = (change_x * std::cos(neg_angle)) - (change_y * std::sin(neg_angle));
  float y_prime = (change_x * std::sin(neg_angle)) + (change_y * std::cos(neg_angle));

  // float delta_angle_base_link = odom_angle - prev_odom_angle_;
  // delta_angle_base_link = 0.025;
  float delta_angle_base_link = AngleDelta(odom_angle, prev_odom_angle_);

  return std::make_tuple(x_prime, y_prime, delta_angle_base_link);
}

struct Particle ParticleFilter::NoiseFreeNextLocation(const Vector2f& odom_loc, const float odom_angle, struct Particle p) {
  std::tuple<float, float, float> delta_base_link = ComputeDeltaBaseLink(odom_loc, odom_angle);

  float x_prime = std::get<0>(delta_base_link);
  float y_prime = std::get<1>(delta_base_link);
  float delta_angle_base_link = std::get<2>(delta_base_link); 

  float prev_map_angle = p.angle;

  // compute R(\theta^{map}_1) * \Delta T^{base_link}
  float map_rotation_x = (x_prime * std::cos(prev_map_angle)) - (y_prime * std::sin(prev_map_angle));
  float map_rotation_y = (x_prime * std::sin(prev_map_angle)) + (y_prime * std::cos(prev_map_angle));

  float prev_map_x = p.loc(0);
  float prev_map_y = p.loc(1);

  // add T^{map}_1 to what we computed above
  float new_map_x = prev_map_x + map_rotation_x;
  float new_map_y = prev_map_y + map_rotation_y;

  // compute the new angle in the map frame
  float new_map_angle = prev_map_angle + delta_angle_base_link;

  struct Particle new_p = {Vector2f(new_map_x, new_map_y), new_map_angle, p.weight};

  return new_p;
}

float ParticleFilter::AngleDelta(float curr_angle, float prev_angle) {
  float diff = curr_angle - prev_angle;

  if (std::abs(diff) > M_PI) {
    if (diff < 0) {
      return diff + (M_PI * 2);
    } else {
      assert(diff < (2*M_PI));
      return (M_PI * 2) - diff;
    }
  }
  return diff;
}

struct Particle ParticleFilter::SampleMotionModel(Vector2f odom_loc, float odom_angle, struct Particle particle) {
  // simple motion model: N(x_t + u_t+1, k * |u_t+1|)
  // assumes particle loc and angle are updated to the noise-free location based on odom_loc and odom_angle

  struct Particle new_particle = particle;
  float change_x = odom_loc(0) - prev_odom_loc_(0);
  float change_y = odom_loc(1) - prev_odom_loc_(1);
  float magnitude = std::sqrt( change_x*change_x + change_y*change_y);
  float change_angle = AngleDelta(odom_angle, prev_odom_angle_);
  float new_x = rng_.Gaussian(particle.loc(0), (K1 * std::abs(magnitude)) + (K2 * change_angle));
  float new_y = rng_.Gaussian(particle.loc(1), (K1 * std::abs(magnitude)) + (K2 * change_angle));
  new_particle.loc = Vector2f(new_x, new_y);

  new_particle.angle = rng_.Gaussian(particle.angle, (K3 * std::abs(magnitude)) + (K4 * change_angle));
  return new_particle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  // TODO: initialize based on some distribution
  // for now, just assume that the provided information is true and initialize all particles 
  // to be at that location

  float x, y;
  for (int i = 0; i < FLAGS_num_particles; i++) {
    x = rng_.Gaussian(loc(0), INIT_STDDEV_T);
    y = rng_.Gaussian(loc(1), INIT_STDDEV_T);
    float theta = rng_.Gaussian(angle, INIT_STDDEV_R);

    // struct Particle p = {loc, angle, 1};
    struct Particle p = {Vector2f(x, y), theta, 1/FLAGS_num_particles};
    particles_.push_back(p);
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:

  loc(0) = 0;
  loc(1) = 0;
  float sin_avg = 0;
  float cos_avg = 0;
  float weight_sum = 0;


  for (size_t i = 0; i < particles_.size(); i++) {
    weight_sum += particles_[i].weight;
    loc(0) += particles_[i].loc(0) * particles_[i].weight;
    loc(1) += particles_[i].loc(1) * particles_[i].weight;
    sin_avg += std::sin(particles_[i].angle) * particles_[i].weight;
    cos_avg += std::cos(particles_[i].angle) * particles_[i].weight;
  }
  sin_avg /= weight_sum;
  cos_avg /= weight_sum;

  loc(0) /= weight_sum;
  loc(1) /= weight_sum;
  angle = std::atan2(sin_avg, cos_avg);

}


}  // namespace particle_filter
