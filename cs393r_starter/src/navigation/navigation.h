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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>
#include <queue>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>

#include "eigen3/Eigen/Dense"
#include "vector_map/vector_map.h"
#include "shared/math/math_util.h"
#include "shared/math/line2d.h"
#include "simple_queue.h"

using geometry::line2f;

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace std {
class Node {
 public:
  int x_;
  int y_;
  float g_;
  
  Node(int x, int y, float g) {
    x_ = x;
    y_ = y;
    g_ = g;
  }

  Node() = default;

  Node(const Node&) = default;

  bool operator==(const Node& other) const {
    return x_ == other.x_ && y_ == other.y_;
  }

  bool operator!=(const Node& other) const {
    return x_ != other.x_ || y_ != other.y_;
  }
};

template <>
struct hash<Node> {
  size_t operator()(const Node& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x_);
    boost::hash_combine(seed, s.y_);
    return seed;
  }
};

class State {
 public:
  int x_;
  int y_;

  State(int x, int y) : x_(x), y_(y) {}
};

class Env {
 public: 
  Node goal_; // g is undefined for goal node
  vector_map::VectorMap* map_;
  float granularity_; // meters
  // unordered_map<Node, vector<line2f>, hash<Node>> map_lines_;
  const float WALL_CLEARANCE = 0.2;

 
  Env(float goal_x, float goal_y, vector_map::VectorMap* map, float gran) : goal_(round(goal_x/gran), round(goal_y/gran), -1), map_(map), granularity_(gran) {
    // create a map that maps the top left of a 10m x 10m square to a list of lines that are in that square
  }

  Env(vector_map::VectorMap* map, float gran) : map_(map), granularity_(gran) {}

  void UpdateGoal(float x, float y) {
    goal_.x_ = ToMu(x);
    goal_.y_ = ToMu(y);
  }

  float ToMeters(int x) {
    return static_cast<float>(x)*granularity_;
  }

  int ToMu(float m) {
    return round(m/granularity_);
  }

  bool CloseToWall(int x, int y, const line2f& wall) {
    float dist = DistToWall(x, y, wall);
    return dist < 0.5;
  }

  float DistToWall(int x, int y, const line2f& wall) {
    // float t_den = (std::pow(wall.p1(0) - wall.p0(0), 2) +
    //                std::pow(wall.p1(1) - wall.p0(1), 2));
    // float t = -1.0 * 
    //               ((wall.p0(0) - x) * (wall.p1(0) - wall.p0(0)) + 
    //               (wall.p0(1) - y) * (wall.p1(1) - wall.p0(1))) / 
    //               t_den;
    // if (t >= 0 && t <= 1) {
    //   float d_num = std::abs((wall.p1(0) - wall.p0(0)) * (wall.p0(1) - y) - (wall.p1(1) - wall.p0(1)) * (wall.p0(0) - x));
    //   float d = d_num / std::pow(t_den, 0.5);
    //   return d;
    // }
    float d1 = std::hypot(wall.p0(0) - x, wall.p0(1) - y);
    float d2 = std::hypot(wall.p1(0) - x, wall.p1(1) - y);
    return std::min(d1, d2);
  }

  bool WallCollision(int x1, int y1, int x2, int y2) {
    // Convert to meters
    line2f edge(ToMeters(x1), ToMeters(y1), ToMeters(x2), ToMeters(y2));

    // can make it run faster later, first just loop through each line in the wall
    for (size_t i = 0; i < map_->lines.size(); i++) {
      if (map_->lines[i].Intersects(edge)) {
        return true;
      }
    }

    return false;
  }

  void GetNeighbors(Node curr, std::vector<Node>& neighbors) {
    int dx[] = {1,1, 1, 0,-1,-1,0,-1};
    int dy[] = {1,0,-1,-1, 1, 0,1,-1};

    for (int i = 0; i < 8; i++) {
      float g_delta = (dx[i] == 0 || dy[i] == 0) ? 1 : 1.41;
      // g_delta = 1;
      if (!WallCollision(curr.x_, curr.y_, curr.x_+dx[i], curr.y_+dy[i])) {
        neighbors.push_back(Node(curr.x_+dx[i], curr.y_+dy[i], curr.g_+g_delta));
      }
    }
  }

  float Heuristic(Node curr) {
    // return 0;
    return std::hypot(static_cast<float>(curr.x_- goal_.x_), static_cast<float>(curr.y_- goal_.y_)) * granularity_;
  }

  bool IsSolution(Node curr) {
    return curr == goal_;
  }
};

class AStar {
 public:
  explicit AStar(vector_map::VectorMap* map, float gran) : env_(map, gran) {}

  explicit AStar(float goal_x, float goal_y, vector_map::VectorMap* map, float gran) : env_(goal_x, goal_y, map, gran) {}

  void UpdateGoal(float x, float y) { 
    env_.UpdateGoal(x, y);
  }

  bool Search(float start_x, float start_y, vector<Eigen::Vector2f>& path) {
    // Set up the values
    std::unordered_map<Node, Node, std::hash<Node>> parent;
    std::unordered_map<Node, float, std::hash<Node>> cost;
    SimpleQueue<Node, float> queue;

    // cout << "map line size: " << env_.map_->lines.size() << endl; 

    // Set up the parent map, cost map, and queue
    Node start(env_.ToMu(start_x), env_.ToMu(start_y), 0);
    cost[start] = start.g_;
    queue.Push(start, start.g_ + env_.Heuristic(start));

    // Add the start node to our priority queue
    while (!queue.Empty()) {

      // Pop off the queue
      Node current_node;
      current_node = queue.Pop();

      // cout << "Expanded node: " << current_node.x_ << " " << current_node.y_ << " " << current_node.g_ << endl;

      // Check to see if the popped node is the solution and return
      if (env_.IsSolution(current_node)) {
        GetPath(path, parent, current_node);
        return true;
      }

      // Loop over the neighbors
      vector<Node> neighbors;
      assert(current_node.g_ == cost[current_node]);
      env_.GetNeighbors(current_node, neighbors);
      for (auto n : neighbors) {
        if ((cost.count(n) == 0) || (n.g_ < cost[n])) {
          cost[n] = n.g_;
          queue.Push(n, n.g_ + env_.Heuristic(n));
          // cout << "\tneighbor: " << n.x_ << " " << n.y_ << " " << n.g_  << " " << env_.Heuristic(n) << " " << n.g_ + env_.Heuristic(n) << endl;
          parent[n] = current_node;
        }
      }
      
    }

    return false;
  }

  void GetPath(
    vector<Eigen::Vector2f>& path, 
    std::unordered_map<Node, Node, std::hash<Node>>& parent,
    Node final_node) {
      Node curr_node;
      curr_node = final_node;
      while (curr_node.g_ != 0) {
        path.push_back(Eigen::Vector2f(env_.ToMeters(curr_node.x_), env_.ToMeters(curr_node.y_)));
        curr_node = parent[curr_node];
      }
      // Add on the start node 
      path.push_back(Eigen::Vector2f(env_.ToMeters(curr_node.x_), env_.ToMeters(curr_node.y_)));
      std::reverse(path.begin(), path.end());
    }
  
  private:
    Env env_;
};



} // namespace std


namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  float carrot_distance;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

enum State {
  AUTO,
  FL,
  RR,
  GOAL,
  SANDBOX
};

struct Control {
  float velocity;
  float curvature;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();

  float DistToGoal();

  void RunAutonomous();

  void RunRobot(float velocity, float curvature);

  void PublishControl();

  void PublishVis();

  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  double PointFreePath(const Eigen::Vector2f& point, const Eigen::Vector2f& turning_center, float small_radius, float mid_radius, float large_radius, float turning_radius);

  bool CollisionCheck(const Eigen::Vector2f& point, const Eigen::Vector2f& turning_center, float small_radius, float large_radius);

  void Initialize(const std::string& map_file,
                  const Eigen::Vector2f& loc,
                  const float angle);

  double FreePathLength(float proposed_curvature);

  float ComputeVelocity(float current_velocity, float free_path);

  void LatencyCompensation();

  float GoStraightFreePath();

  std::vector<float> ProposeCurvatures();

  std::pair<PathOption, float> PickCurve(std::vector<float> proposed_curves);

  PathOption RewardFunction(std::vector<PathOption> path_options);

  float ApplyRewardFunction(PathOption option);

  double CalculateClearance(float proposed_curvature);

  float ComputeCarrotDistance(float proposed_curvature);

  void DoPointUpdate(void);

 private:

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  std::queue<struct Control> past_controls_;

  float prev_curv_;

  State state_;

  int j_turn_timer_;

  struct Control current_control_;

  vector_map::VectorMap map_;

  std::AStar global_planner_;

  std::vector<Eigen::Vector2f> global_path_;

  size_t path_index_; // must be reset whenever we replan

  bool plan_success_;

  float last_curv_;

};

}  // namespace navigation

#endif  // NAVIGATION_H
