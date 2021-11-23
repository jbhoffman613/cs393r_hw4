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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <cassert>
#include <fstream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Vector3f;
using Eigen::MatrixXf;
using Eigen::Rotation2D;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using std::ofstream;
using std::min;
using std::max;
using vector_map::VectorMap;
using PCloudMat = Eigen::Matrix<float, 3, Eigen::Dynamic>;

const float LOOKUP_METERS = 20.0;
const float LOOKUP_PIXEL_DIM = 0.02; // dimensions of each "pixel" in the lookup table
const int LOOKUP_AREA = 10;
const float LOOKUP_DIM = LOOKUP_METERS / LOOKUP_PIXEL_DIM; // dimensions of the lookup table
const float LOOKUP_STDDEV = 2; // IN CENTIMETERS 
const float LOOKUP_MEAN = 0.0; // IN CENTIMETERS
const float LIDAR_OFFSET = 0.205;
const int CUBE_SIDE_LENGTH = 11; //Keep it an odd number so that way it is symmetrical
const float PERTURB_SIZE = 0.02; // In meters equalling the perturbation step size for our cube
const float PERTURB_RADIAN = 0.0174533*5; // In radians equalling the perturbation step size for our cube - currently at 1 degree
const float X_STD_DEV = 0.3;
const float Y_STD_DEV = 0.3;
const float THETA_STD_DEV = 0.3;

const float K1 = 0.05;
const float K2 = 0.05;
const float K3 = 0.5; 
const float K4 = 1; 

bool print = false;
int pose_count = 0;


namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

Eigen::Affine2f DeltaToTransform(float dx, float dy, float dt) {
  Eigen::Affine2f m = Eigen::Affine2f::Identity();
  m.rotate(dt).pretranslate(Eigen::Vector2f(dx, dy));
  return m;
}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // if (poses_.size() > 2) {
  //   exit(1);
  // }
  if (poses_.empty()) {
    return;
  }
  // Go over all poses to get the current value 
  // poses_.push_back(PoseTriangle(Affine2f::Identity(), prev_odom_loc_, prev_odom_angle_, point_cloud));
  Affine2f current_affine = Affine2f::Identity();
  //  Affine2f current_affine = poses_[0].curr_to_prev_.inverse();
  for (auto p_i : poses_) {
    // current_affine = current_affine * p_i.curr_to_prev_;
    current_affine = p_i.curr_to_prev_ * current_affine;
  }
  // current_affine =  current_affine * poses_[0].curr_to_prev_.inverse();
  *loc = current_affine.translation();
  MatrixXf rotation_matrix = current_affine.rotation();
  float cos_theta = std::acos(rotation_matrix(0,0));
  float sin_theta = std::asin(rotation_matrix(1,0));
  if (sin_theta >= 0) {
    *angle = cos_theta;
  } else {
    *angle = -1.0 * cos_theta;
  }
}

// we can evaluate a zero-mean gaussian for the motion model
// params: perturbation from odom_delta
// return: log likelihood of this motion (farther from odom_delta means smaller likelihood)
float SLAM::EvaluateMotionModel(Vector3f odom_delta, float dx, float dy, float d_theta) {
  // evaluate dx, dy, d_theta in a zero mean gaussian

  // right now this uses a tunable std dev. it should really be based on the magnitude of 
  // the transformation since the last pose. but since we take a new pose every 50 cm or every 30
  // degrees, these magnitudes will not vary a huge amount so this is probably reasonable 
  // for now

  // TODO: base std dev on magnitude of transform, rather than tunable constants
  float std_dev_translation = K1 * std::sqrt(odom_delta(0)*odom_delta(0) + odom_delta(1)*odom_delta(1)) + K2 * odom_delta(3);
  float std_dev_angle = K3 * std::sqrt(odom_delta(0)*odom_delta(0) + odom_delta(1)*odom_delta(1)) + K4 * odom_delta(3);  
  float likelihood_dx = EvaluateLogGaussian(0, std_dev_translation, dx);
  float likelihood_dy = EvaluateLogGaussian(0, std_dev_translation, dy);
  float likelihood_dtheta = EvaluateLogGaussian(0, std_dev_angle, d_theta);

  return likelihood_dx + likelihood_dy + likelihood_dtheta;
}

// transform the curr_cloud
// params: pertubation from odom_delta, lookup table, new point cloud
float SLAM::ObservationLikelihood(const vector<vector<float>>& lookup_table, 
                                  const Affine2f& perturb, 
                                  const Affine2f& odom_delta,
                                  const MatrixXf& curr_cloud) {
  
  // cout << "curr cloud point: " << curr_cloud[0](0) << ", " << curr_cloud[0](1) << ", " << curr_cloud[0](2) << endl;
  // transform curr cloud to same frame as prev_pose_ by using the odom_delta and the perturb.
  // MatrixXf curr_cloud_matrix(3, curr_cloud.size());
  // for (size_t i = 0; i < curr_cloud.size(); i++) {
  //   for (int j = 0; j < 3; j++) {
  //     curr_cloud_matrix(j, i) = curr_cloud[i][j];
  //   }
  // } 

  auto curr_cloud_prev_frame = (odom_delta.matrix() * perturb.matrix()) * curr_cloud;


  float likelihood = 0;
  for (int i = 0; i < curr_cloud.cols(); i++) {
    Vector2i idxs = Point2Index(Vector2f(curr_cloud_prev_frame(0, i), curr_cloud_prev_frame(1, i)));
    // for each point in the transformed curr_cloud, take the value from the lookup table as the likelihood
    if (idxs(0) >= 0 && idxs(0) < LOOKUP_DIM && idxs(1) >= 0 && idxs(1) < LOOKUP_DIM) {
      likelihood += lookup_table[idxs(0)][idxs(1)];
    }  
  }

  // if using log likelihood, add them all up and return value as your likelihood
  return likelihood;
}

// creates a lookup table from point cloud by plopping a gaussian on each point 
// and creating an image
// return: lookup table
// assume the given point cloud is in the lidar frame
// assume point cloud is in lidar frame
// vector<vector<float>> SLAM::LookupTable(const vector<Vector2f>& point_cloud) {
vector<vector<float> > SLAM::LookupTable(const MatrixXf& point_cloud) {

  vector<vector<float> > lookup_table(LOOKUP_DIM, vector<float>(LOOKUP_DIM, -100.0));

  // plop a gaussian on each point
  for (int i = 0; i < point_cloud.cols(); i++) {
    float point_x = point_cloud(0, i);
    float point_y = point_cloud(1, i);

    Vector2i point_idx = Point2Index(Vector2f(point_x, point_y));
    int x_cm_rounded = point_idx(0);
    int y_cm_rounded = point_idx(1);

    if (x_cm_rounded > LOOKUP_DIM || x_cm_rounded < 0 || y_cm_rounded > LOOKUP_DIM || y_cm_rounded < 0) {
      continue;
    }

    vector<Vector2i> surrounding_points;
    float area_x = x_cm_rounded - LOOKUP_AREA;
    float area_y = y_cm_rounded - LOOKUP_AREA;
    for (int j = 0; j <= LOOKUP_AREA * 2; j++) {
      float cur_proposal_x = area_x;
      for (int k = 0; k <= LOOKUP_AREA * 2; k++) {
        float zero = 0.0;
        int to_append_x = min(max(cur_proposal_x, zero), LOOKUP_DIM - 1);
        int to_append_y = min(max(area_y, zero), LOOKUP_DIM -1);
        surrounding_points.push_back(Vector2i(to_append_x, to_append_y));
        cur_proposal_x++;
      }
      area_y++;
    }

    for (auto j : surrounding_points) {
      float x = j(0);
      float y = j(1);
      float gaussian_distance = std::hypot(x_cm_rounded - x, y_cm_rounded - y);
      float evaluated_gauss_value = EvaluateLogGaussian(LOOKUP_MEAN, LOOKUP_STDDEV, gaussian_distance);
      lookup_table[x][y] = std::max(lookup_table[x][y], evaluated_gauss_value);
    }
  }
  
  return lookup_table;
}

Vector2i SLAM::Point2Index(Vector2f point) {
  float point_x = point(0);
  float point_y = point(1);

  float x_img_adjusted = 10 + point_x;
  float y_img_adjusted = 10 - point_y;

  float x_cm = x_img_adjusted * 100;
  float y_cm = y_img_adjusted * 100;

  // divide by 2 and round to get an index
  int x_cm_rounded = round(x_cm / 2);
  int y_cm_rounded = round(y_cm / 2);

  return Vector2i(x_cm_rounded, y_cm_rounded);
}

// params: prev pose, prev point cloud, curr point cloud, delta odom
// return transformation that corresponds to curr pose
Affine2f SLAM::ScanMatch(const PoseTriangle& prev_pose, 
                        const MatrixXf& curr_cloud, 
                        const Affine2f& odom_delta) {

  // get lookup table for previous pose
  vector<vector<float> > lookup_table = LookupTable(prev_pose.point_cloud_);

  // create cube of perturbations (add hyperparameters)

  // construct empty cube
  vector<vector<vector<float> > > cube = MakeCube();

  float max_likelihood = -10000000000;
  Vector3f mlp_vec;
  Affine2f max_likelihood_perturb;
  // Affine2f odom_delta_affine = DeltaToTransform(odom_delta(0), odom_delta(1), odom_delta(2));
  Vector2f odom_translation = odom_delta.translation();
  MatrixXf odom_rotation_matrix = odom_delta.rotation();
  float cos_theta = std::acos(odom_rotation_matrix(0,0));
  float sin_theta = std::asin(odom_rotation_matrix(1,0));
  float odom_rotation;
  if (sin_theta >= 0) {
    odom_rotation = cos_theta;
  } else {
    odom_rotation = -1.0 * cos_theta;
  }
  Vector3f odom_delta_vec(odom_translation(0), odom_translation(1), odom_rotation);

  // walk through all entries in the cube
  for (int x_index = 0; x_index < CUBE_SIDE_LENGTH; x_index++) {
    for (int y_index = 0; y_index < CUBE_SIDE_LENGTH; y_index++) {
      for (int theta_index = 0; theta_index < CUBE_SIDE_LENGTH; theta_index++) {
        float dx = IndexToDVal(x_index);
        float dy = IndexToDVal(y_index);
        float d_theta = IndexToDRad(theta_index);
        Affine2f perturb = DeltaToTransform(dx, dy, d_theta);
        // float motion_model_likelihood = EvaluateMotionModel(dx, dy, d_theta);
        float motion_model_likelihood = EvaluateMotionModel(odom_delta_vec, dx, dy, d_theta);
        // The fourth loop is in the ObsLikeli func below - this makes it equivalent to the naive approach from the CSM paper
        float observation_likelihood = ObservationLikelihood(lookup_table, perturb, odom_delta, curr_cloud);
        // adding log likelihoods together
        cube[x_index][y_index][theta_index] = motion_model_likelihood + observation_likelihood;
        if (cube[x_index][y_index][theta_index] > max_likelihood) {
          max_likelihood = cube[x_index][y_index][theta_index];
          max_likelihood_perturb = perturb;
          mlp_vec = Vector3f(dx, dy, d_theta);
        }
      }
    }
  }
  // cout << "max likelihood perturb: " << mlp_vec(0) << ", " << mlp_vec(1) << ", " << mlp_vec(2) << endl;

  // Affine2f perturbed = odom_delta * max_likelihood_perturb;
  // for (int i = 0; i < perturbed.rows(); i++ ){
  //   for (int j = 0; j < perturbed.cols(); j++) {
  //     cout << perturbed(i,j) << " ";
  //   }
  //   cout << endl;
  // }
  // TODO THE ORDER MAY BE WRONG
  cout << mlp_vec(0) << ", " << mlp_vec(1) << " " << mlp_vec(2) << endl;
  return odom_delta * max_likelihood_perturb;
  // return odom_delta;
}

bool SLAM::ObsLikTest(MatrixXf curr_point_cloud) {
  // first take the point cloud and rotate it 5 degrees to get a new point cloud
  
  Affine2f rot5deg = DeltaToTransform(0.06, -0.02, 0.0174533*-3);
  MatrixXf prev_pt_cloud = rot5deg.matrix() * curr_point_cloud; 

  vector<vector<float>> prev_lookup = LookupTable(prev_pt_cloud);
  
  vector<vector<vector<float> > > cube = MakeCube();

  cout << "THERE IS A CUBE" << endl;

  float max_likelihood = -10000000000;
  Vector3f max_likelihood_perturb(0, 0, 0);
  // walk through all entries in the cube

  for (int x_index = 0; x_index < CUBE_SIDE_LENGTH; x_index++) {
    for (int y_index = 0; y_index < CUBE_SIDE_LENGTH; y_index++) {
      for (int theta_index = 0; theta_index < CUBE_SIDE_LENGTH; theta_index++) {
        float dx = IndexToDVal(x_index);
        float dy = IndexToDVal(y_index);
        float d_theta = IndexToDRad(theta_index);
        float observation_likelihood = ObservationLikelihood(prev_lookup, DeltaToTransform(dx, dy, d_theta), Affine2f::Identity(), curr_point_cloud);
        cube[x_index][y_index][theta_index] = observation_likelihood;
        if (cube[x_index][y_index][theta_index] > max_likelihood) {
          max_likelihood = cube[x_index][y_index][theta_index];
          max_likelihood_perturb = Vector3f(dx, dy, d_theta);
        }
      }
    }
  }
  cout << "max likelihood perturb: " << endl;
  cout << "likelihood: " << max_likelihood << endl;
  cout << "x: " << max_likelihood_perturb(0) << endl;
  cout << "y: " << max_likelihood_perturb(1) << endl;
  cout << "theta: " << max_likelihood_perturb(2) << endl;

  return true;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max,
                        float angle_increment) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  Affine2f odom_delta;

  // we determine when to take a new pose in ObserveOdometry
  if (pose_) {

    // Location of the laser on the robot. Assumes the laser is forward-facing.
    const Vector2f kLaserLoc(0.2, 0);

    // vector<Vector3f> point_cloud;
    MatrixXf point_cloud(3, ranges.size());

    int num_measurements = (angle_max - angle_min) / angle_increment + 1;

    for (int i = 0; i < num_measurements; i++) {
      // converting from polar laser scan coords to cartesian
      float r = ranges[i];
      // if (r > 0.25 && r < 10.0) {
        float theta = angle_min + angle_increment * i;
        point_cloud(0, i) = r*std::cos(theta) + LIDAR_OFFSET;
        point_cloud(1, i) = r*std::sin(theta);
        point_cloud(2, i) = 1;
        // point_cloud.push_back(Vector3f(r*std::cos(theta) + LIDAR_OFFSET, r*std::sin(theta), 1));   
      // }
    }

    if (poses_.empty()){ 
      // prev_odom_loc_ = Vector2f(3, 4);
      // prev_odom_angle_ = 0.0;
      // Affine2f first_pose_transform = DeltaToTransform(prev_odom_loc_(0), prev_odom_loc_(1), prev_odom_angle_);
      // cout << "FIRST POSE AT " << prev_odom_loc_(0) << "," << prev_odom_loc_(1) << " " << prev_odom_angle_ << endl;
      // poses_.push_back(PoseTriangle(first_pose_transform, prev_odom_loc_, prev_odom_angle_, point_cloud));
      // poses_.push_back(PoseTriangle(Affine2f::Identity(), prev_odom_loc_, prev_odom_angle_, point_cloud));

      poses_.push_back(PoseTriangle(Affine2f::Identity(), prev_odom_loc_, prev_odom_angle_, point_cloud));

    } else {
      // // auto PoseTriangle
      // // poses_.push_back(PoseTriangle(prev_odom_loc_(0) - ))
      // PoseTriangle prev_pose = poses_.back();
      
      // float dt = AngleDelta(prev_odom_angle_, prev_pose.odom_angle_);
      // cout << "odometry at pose: " << prev_odom_loc_(0) << ", " << prev_odom_loc_(1) << " " << prev_odom_angle_ << endl;
      // odom_delta = DeltaToTransform(dx, dy, dt);
      // Affine2f delta = ScanMatch(prev_pose, point_cloud, odom_delta);

      // poses_.push_back(PoseTriangle(delta, prev_odom_loc_, prev_odom_angle_, point_cloud));

      PoseTriangle prev_pose = poses_.back();
      float dx = prev_odom_loc_(0) - prev_pose.odom_loc_(0);
      float dy = prev_odom_loc_(1) - prev_pose.odom_loc_(1);
      // float change_x = odom_loc(0) - prev_odom_loc_(0);
      // float change_y = odom_loc(1) - prev_odom_loc_(1);
      // float neg_angle = -1.0 * prev_odom_angle_;

      float neg_angle = -1.0 * prev_pose.odom_angle_;

      // // compute R(-\theta^{odom}_1)*(T^{odom}_2 - T^{odom}_1) for x and y
      // float x_prime = (change_x * std::cos(neg_angle)) - (change_y * std::sin(neg_angle));
      // float y_prime = (change_x * std::sin(neg_angle)) + (change_y * std::cos(neg_angle));
      float x_prime = (dx * std::cos(neg_angle)) - (dy * std::sin(neg_angle));
      float y_prime = (dx * std::sin(neg_angle)) + (dy * std::cos(neg_angle));

      // // float delta_angle_base_link = odom_angle - prev_odom_angle_;
      // // delta_angle_base_link = 0.025;
      float delta_angle_base_link = AngleDelta(prev_odom_angle_, prev_pose.odom_angle_);

      Affine2f odom_delta = DeltaToTransform(x_prime, y_prime, delta_angle_base_link);
      
      Affine2f delta = ScanMatch(prev_pose, point_cloud, odom_delta);
      poses_.push_back(PoseTriangle(delta, prev_odom_loc_, prev_odom_angle_, point_cloud));
    }

    pose_ = false;
  }
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    // initial_odom_loc_ = odom_loc;
    // initial_odom_angle_ = odom_angle;
    // first_pose_to_map_ = DeltaToTransform(odom_loc(0), odom_loc(1), odom_angle);
    dist_change_for_pose_ = 0;
    angle_change_for_pose_ = 0;
    pose_ = true;
    return;
  }
  // cout << "odom loc: " << odom_loc(0) << "," << odom_loc(1) << endl;
  // cout << "odom angle: " << odom_angle << endl;
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
  float change_x = odom_loc(0) - prev_odom_loc_(0);
  float change_y = odom_loc(1) - prev_odom_loc_(1);
  float change_angle = AngleDelta(odom_angle, prev_odom_angle_);

  float distance_traveled = std::sqrt(change_x*change_x + change_y*change_y);

  dist_change_for_pose_ += distance_traveled;
  angle_change_for_pose_ += std::abs(change_angle);

  // if the robot has moved more than 50 cm or turned more than 30 degrees, 
  // make a note that it's time to take a new pose
  if (dist_change_for_pose_ > 0.5 || 
    angle_change_for_pose_ > 0.52) { // 0.52 radians is roughly 30 degrees
    pose_ = true;
    dist_change_for_pose_ = 0;
    angle_change_for_pose_ = 0;
  }

  prev_odom_angle_ = odom_angle;
  prev_odom_loc_ = odom_loc;
}

float SLAM::AngleDelta(float curr_angle, float prev_angle) const {
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

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;

  Affine2f current_affine = Affine2f::Identity();
  //  Affine2f current_affine = poses_[0].curr_to_prev_.inverse();
  // P_I is a pose_trinagle
  for (auto p_i : poses_) {
    
    // TODO MAY NEED TO FLIP THE ORDER HERE
    current_affine = p_i.curr_to_prev_ * current_affine;
    // current_affine = current_affine * p_i.curr_to_prev_;

    MatrixXf transformed_scan = current_affine.matrix() * p_i.point_cloud_;

    for (int ray = 0; ray < p_i.point_cloud_.cols(); ray++) {
      Vector2f car_loc = current_affine.translation();
      // get the distance between the point and the location of the car that the 
      // point was observed at. if it's outside the range of the lidar, drop it
      float dist = std::sqrt( (car_loc(0) - transformed_scan(0, ray))*(car_loc(0) - transformed_scan(0, ray)) + 
          (car_loc(1) - transformed_scan(1, ray))*(car_loc(1) - transformed_scan(1, ray)));
      if (dist > 0.25 && dist < 10.0) {
        map.push_back(Vector2f(transformed_scan(0, ray), transformed_scan(1, ray)));
      }
    }
  }
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}


// Consumes a mean, a std_dev, and a distance, and returns 
// a float of the distance evaluated for that gaussian
// expects ALL arguments in CENTIMETERS
float SLAM::EvaluateLogGaussian(float mean, float std_dev, float distance) {
  float std_dev_const = 1 / (std_dev * std::sqrt(2 * M_PI));
  float exp_part = (distance - mean) / std_dev;
  float exponent = -0.5 * std::pow(exp_part, 2);
  float final_val = std_dev_const * exp(exponent);
  return std::log(final_val);
}

// Takes an index from a box, and returns the perturbation value for that index. 
// The D value is in meters. 
float SLAM::IndexToDVal(int index) {
  float d_val = (index - int(floor(CUBE_SIDE_LENGTH / 2))) * PERTURB_SIZE;
  return d_val;
}

// Takes an index from a box, and returns the perturbation value in radians for that index. 
// The D value is in radians. 
float SLAM::IndexToDRad(int index) {
  // NOTE: THIS WILL RETURN A SET OF POSITIVE AND NEGATIVE RADIANS! 
  // May need to add 2*pi to all values that are less than zero. 
  float d_val = (index - int(floor(CUBE_SIDE_LENGTH / 2))) * PERTURB_RADIAN;
  return d_val;
}

// Creates a cube according to the constant CUBE_SIDE_LENGTH full of float zero values. 
vector<vector<vector<float>>> SLAM::MakeCube() {
  // Create the cube object
  vector<vector<vector<float>>> final_cube;
  
  for (int i = 0; i < CUBE_SIDE_LENGTH; i++) {
    // Create the temporary slide to append 
    vector<vector<float>> temporary_slice;

    for (int j = 0; j < CUBE_SIDE_LENGTH; j++) {
      // Create the single line of a slice to append, and append it.
      vector<float> oned_vector(CUBE_SIDE_LENGTH, 0.0);
      temporary_slice.push_back(oned_vector);
    }
    
    // Append the slice of the cube to the cube. 
    final_cube.push_back(temporary_slice);
  }
  return final_cube;
}

}  // namespace slam
