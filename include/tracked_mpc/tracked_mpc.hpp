#ifndef MPC_HPP
#define MPC_HPP

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

// #include "./cvxgen/ldl.c"
// #include "./cvxgen/matrix_support.c"
// #include "./cvxgen/solver.c"
#include "./cvxgen/solver.h"

namespace tracked_mpc {
class TrackedMPC {
public:
  TrackedMPC(const double v, const double dt) : v_(v), dt_(dt) {
    params.A[0] = 1;
    params.A[1] = v_ * dt_;
    params.A[2] = 0;
    params.A[3] = 1;
    params.B[0] = 0;
    params.B[1] = 1;
  }
  ~TrackedMPC() {}
  void calcU(const nav_msgs::Path path, const geometry_msgs::Pose current_pose,
             const geometry_msgs::Twist current_twist, const int target_index, const double dt) {
    const int path_size = path.poses.size();
    setInitState(current_pose, current_twist);
    // N点先までの目標軌道の設定
    for (int i = 0; i < N_; ++i) {
      if (target_index + i < path_size) {
        params.x_ref[i][0] = path.poses[target_index + i].pose.position.x;
        params.x_ref[i][1] = path.poses[target_index + i].pose.position.y;
        params.x_ref[i][2] = path.poses[target_index + i].pose.orientation.w;
      } else {
        params.x_ref[i][0] = path.poses[path_size - 1].pose.position.x;
        params.x_ref[i][1] = path.poses[path_size - 1].pose.position.y;
        params.x_ref[i][2] = path.poses[path_size - 1].pose.orientation.w;
      }
    }
    solve();
    twist_.linear.x = vars.u[1][0];
    twist_.linear.y = vars.u[1][1];
    twist_.angular.z = vars.u[1][2];
  }
  void setInitState(const geometry_msgs::Pose current_pose,
                    const geometry_msgs::Twist current_twist) {
    params.x_0[0] = current_pose.position.x;
    params.x_0[1] = current_pose.position.y;
    params.x_0[2] = current_pose.orientation.w;
  }
  geometry_msgs::Twist getU() { return twist_; }

private:
  int N_;
  double dt_, v_;
  Vars vars;
  Params params;
  Workspace work;
  Settings settings;
  geometry_msgs::Twist twist_;
};
} // namespace tracked_mpc
#endif