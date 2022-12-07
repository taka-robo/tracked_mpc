#include <geometry_msgs/Pose.h>
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

class DummyBot {
public:
  DummyBot() {
    ros::NodeHandle nh;
    // parameter
    parent_frame_ = nh.param<std::string>("parent_frame", "world");
    child_frame_ = nh.param<std::string>("child_frame", "true/rover0");
    const ros::Rate rate(nh.param("rate", 10.));
    x_ = nh.param("init_x", 0);
    y_ = nh.param("init_y", 0);
    z_ = nh.param("init_z", 0);
    th_ = nh.param("init_th", 0);
    vx_ = vy_ = vz_ = 0;
    // Publisher
    tf_pub_.reset(new tf2_ros::StaticTransformBroadcaster());
    // Subscriber
    sub_twist_ = nh.subscribe("cmd_vel", 1000, &DummyBot::twistDataCallback, this);
    timer_ = nh.createTimer(rate.expectedCycleTime(), &DummyBot::timerUpdateCallback, this);
  }
  ~DummyBot() {}
  static double normalizeRad(const double rad) {
    if (rad > M_PI) {
      return std::fmod(rad + M_PI, 2. * M_PI) - M_PI;
    } else if (rad < -M_PI) {
      return std::fmod(rad - M_PI, 2. * M_PI) + M_PI;
    } else {
      return rad;
    }
  }
  static geometry_msgs::TransformStamped makeTransform(const ros::Time &stamp,
                                                       const std::string &parent,
                                                       const std::string &child, const double x,
                                                       const double y, const double th) {
    geometry_msgs::TransformStamped p2c;
    p2c.header.stamp = stamp;
    p2c.header.frame_id = parent;
    p2c.child_frame_id = child;
    p2c.transform.translation.x = x;
    p2c.transform.translation.y = y;
    p2c.transform.translation.z = 0.;
    p2c.transform.rotation.w = std::cos(th / 2.);
    p2c.transform.rotation.x = 0.;
    p2c.transform.rotation.y = 0.;
    p2c.transform.rotation.z = std::sin(th / 2.);
    return p2c;
  }
  void timerUpdateCallback(const ros::TimerEvent &event) {
    const double dt = (event.current_real - event.last_real).toSec();
    x_ += vx_ * dt * std::cos(th_);
    y_ += vx_ * dt * std::sin(th_);
    th_ = normalizeRad(th_ + w_ * dt);
    tf_pub_->sendTransform(
        makeTransform(event.current_real, parent_frame_, child_frame_, x_, y_, th_));
    // ROS_INFO_STREAM("x:" << x_ << ",y:" << y_ << ",th:" << th_);
  }
  void twistDataCallback(const geometry_msgs::Twist::ConstPtr twist) {
    vx_ = twist->linear.x;
    vy_ = twist->linear.y;
    vz_ = twist->linear.z;
    w_ = twist->angular.z;
    // ROS_INFO_STREAM("vx:" << vx_ << ",w:" << w_);
  }

private:
  double vx_, vy_, vz_, w_;
  double x_, y_, z_, th_;
  ros::Timer timer_;
  ros::Subscriber sub_twist_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_pub_;
  std::string parent_frame_;
  std::string child_frame_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "dummy_bot_node");
  DummyBot bot;
  ros::spin();
  return 0;
}