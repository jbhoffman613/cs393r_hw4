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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

using Eigen::MatrixXf;
using Eigen::Affine2f;

namespace slam {

struct PoseTriangle {
  // Vector3f delta_from_start; // transform from this frame to global frame
  Affine2f curr_to_prev_;
  MatrixXf point_cloud_; // 3 x n matrix of n points from point cloud

  Eigen::Vector2f odom_loc_;
  float odom_angle_;

  // curr_to_prev transformation matrix between the last pose and the current pose 
  // Raw odometry location {x, y}
  // Raw odometry angle from that pose
  // Point cloud from that pose

  PoseTriangle(Affine2f curr_to_prev, 
              Eigen::Vector2f odom_loc, 
              float odom_angle,
              MatrixXf point_cloud) {
    curr_to_prev_ = curr_to_prev;
    odom_loc_ = odom_loc;
    odom_angle_ = odom_angle;
    point_cloud_ = point_cloud;
  }

  // Eigen::Affine2f GetTransFromPrev() {
  //   // return DeltaToTransform(delta_from_prev_(0),
  //   //                         delta_from_prev_(1),
  //   //                         delta_from_prev_(2));
  //   Eigen::Affine2f m = Eigen::Affine2f::Identity();
  //   m.rotate(delta_from_prev_(3)).pretranslate(Eigen::Vector2f(delta_from_prev_(0), delta_from_prev_(1)));
  //   return m;
  // }

  // // Eigen::Affine2f GetTransFromStart() {
  // //   return DeltaToTransform(delta_from_start(0),
  // //                           delta_from_start(1),
  // //                           delta_from_start(2));
  // // }
};  

class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max,
                    float angle_increment);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  bool ObsLikTest(MatrixXf point_cloud);

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

  std::vector<std::vector<float>> LookupTable(const MatrixXf& point_cloud);

  Eigen::Affine2f ScanMatch(const PoseTriangle& prev_pose,
                        const MatrixXf& curr_cloud, 
                        const Eigen::Affine2f& odom_delta);

  float EvaluateMotionModel(Eigen::Vector3f odom_delta, float dx, float dy, float d_theta);

  float ObservationLikelihood(const std::vector<std::vector<float>>& lookup_table, 
                            const Eigen::Affine2f& perturb, 
                            const Eigen::Affine2f& odom_delta, 
                            const MatrixXf& curr_cloud);

  float EvaluateLogGaussian(float mean, float std_dev, float distance); 
  Eigen::Vector2i Point2Index(Eigen::Vector2f point);

  float AngleDelta(float curr_angle, float prev_angle) const;

  std::vector<std::vector<std::vector<float>>> MakeCube();
  float IndexToDVal(int i);
  float IndexToDRad(int index);

 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  // Eigen::Vector2f initial_odom_loc_;
  // float initial_odom_angle_;
  // Eigen::Affine2f first_pose_to_map_;

  bool pose_;
  float dist_change_for_pose_;
  float angle_change_for_pose_;

  std::vector<PoseTriangle> poses_;

  Eigen::Affine2f prev_pose_;
};
}  // namespace slam

#endif   // SRC_SLAM_H_
