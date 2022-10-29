#ifndef TF_UTILS_HPP
#define TF_UTILS_HPP

#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <tf2_ros/buffer.h>

#include <boost/range/begin.hpp>
#include <boost/range/numeric.hpp>  // for boost::accumulate()
#include <boost/range/size.hpp>
#include <boost/range/value_type.hpp>

#include <Eigen/Core>

template <class FrameRange>
static inline bool waitFrames(tf2_ros::Buffer* const buffer, const FrameRange& frames, const ros::Duration& rel_timeout)
{
  const ros::Time abs_timeout = ros::Time::now() + rel_timeout;
  for (const std::string& child : frames)
  {
    if (!buffer->canTransform(*boost::begin(frames), child, ros::Time(0), abs_timeout - ros::Time::now()))
    {
      return false;
    }
  }
  return true;
}

static inline geometry_msgs::Pose lookupPose2d(tf2_ros::Buffer* const buffer, const std::string& parent_frame,
                                             const std::string& child_frame, const ros::Time& target_time)
{
  geometry_msgs::Pose pose;
	geometry_msgs::TransformStamped tf=
      buffer->lookupTransform(parent_frame, child_frame, target_time, ros::Duration(1.));
	pose.position.x = tf.transform.translation.x;
	pose.position.y = tf.transform.translation.y;
	pose.position.z = tf.transform.translation.z;
	pose.orientation = tf.transform.rotation;
  return pose;
}

static inline geometry_msgs::TransformStamped makeTransform(const ros::Time& stamp, const std::string& parent_frame,
                                                            const std::string& child_frame, const double x,
                                                            const double y, const double z)
{
  geometry_msgs::TransformStamped ret;
  ret.header.stamp = stamp;
  ret.header.frame_id = parent_frame;
  ret.child_frame_id = child_frame;
  ret.transform.translation.x = x;
  ret.transform.translation.y = y;
  ret.transform.translation.z = z;
  ret.transform.rotation.w = 1.;
  ret.transform.rotation.x = 0.;
  ret.transform.rotation.y = 0.;
  ret.transform.rotation.z = 0.;
  return ret;
}

template <class MConstPtrRange>
static inline ros::Time averageStamps(const MConstPtrRange& msgs)
{
  using MConstPtr = typename boost::range_value<MConstPtrRange>::type;
  ros::Time stamp;
  stamp.fromNSec(boost::accumulate(
                     msgs, std::uint64_t(0),
                     [](const std::uint64_t sum, const MConstPtr& msg) { return sum + msg->header.stamp.toNSec(); }) /
                 boost::size(msgs));
  return stamp;
}

#endif