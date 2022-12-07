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
  TrackedMPC() {}
  ~TrackedMPC() {}
  void calcU(const nav_msgs::Path path, const geometry_msgs::Pose current_pose,
             const geometry_msgs::Twist current_twist, const int target_index, const double dt) {
    setInitState(current_pose, current_twist);
    // N点先までの目標軌道の設定
    for (int i = 0; i < N_; ++i) {
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
  Vars vars;
  Params params;
  Workspace work;
  Settings settings;
  geometry_msgs::Twist twist_;
};
} // namespace tracked_mpc
#endif