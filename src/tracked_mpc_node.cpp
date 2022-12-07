#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "../include/tracked_mpc/tracked_mpc.hpp"

#include "tf_utils.hpp"

// global variables
ros::Publisher g_cmd_vel_pub;
nav_msgs::Path g_path;
tf2_ros::Buffer tf_buf;

double getDistance(const geometry_msgs::Pose p1, const geometry_msgs::Pose p2) {
  return std::sqrt((p1.position.x - p2.position.x) * (p1.position.x - p2.position.x) +
                   (p1.position.y - p2.position.y) * (p1.position.y - p2.position.y) +
                   (p1.position.z - p2.position.z) * (p1.position.z - p2.position.z));
}
int getNearestPoseIndex(const nav_msgs::Path &path, const geometry_msgs::Pose current_pose) {
  const int path_size = path.poses.size();
  ROS_ASSERT(path_size > 0);
  int nearest_pose_index = 0;
  double min_dist = std::numeric_limits<double>::infinity();
  // search nearest pose index
  for (int index_id = 0; index_id < path_size; ++index_id) {
    const double distance = getDistance(path.poses[index_id].pose, current_pose);
    if (min_dist > distance) {
      min_dist = distance;
      nearest_pose_index = index_id;
    }
  }
  return nearest_pose_index;
}
// Timer call back function for MPC controller
void timerCallback(const ros::TimerEvent event) {
  ROS_ASSERT(g_path.poses.size() > 0);
  const ros::Time target_time = event.current_expected.now();
  const geometry_msgs::Pose current_pose = lookupPose2d(&tf_buf, "map", "body_link", target_time);
  getNearestPoseIndex(g_path, current_pose);
}
// Update tracking path
void pathDataCallback(const nav_msgs::Path::ConstPtr path) {
  g_path.header = path->header;
  g_path.poses = path->poses;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tracked_mpc_node");
  ros::NodeHandle nh;
  // Param
  const ros::Rate control_rate(ros::param::param("~control_rate", 1.));
  // Publisher
  g_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  // Subscriber
  ros::Subscriber sub_path = nh.subscribe("path", 1000, pathDataCallback);
  tf2_ros::TransformListener tf_sub(tf_buf, nh);
  // Timer
  ros::Timer timer = nh.createTimer(control_rate.expectedCycleTime(), timerCallback);

  ros::spin();
}
