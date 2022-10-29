#include <ros/assert.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tracked_mpc/mpc.hpp>
#include <tf_utils.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// global variables
ros::Publisher g_cmd_vel_pub;
nav_msgs::Path g_path;
tf2_ros::Buffer tf_buf;

// Timer call back function for MPC controller
void timerCallback(const ros::TimerEvent event){
	ROS_ASSERT(g_path.poses.size()>0);	

	// getNearestPoseIndex(g_path,);
}
// Update tracking path
void pathDataCallback(const nav_msgs::Path::ConstPtr path){	
	g_path.header = path->header;
	g_path.poses = path->poses;
}
int getNearestPoseIndex(const nav_msgs::Path& path,const geometry_msgs::Pose current_pose){
	int index_id;
	ROS_ASSERT(path.poses.size()>0);
	//search nearest pose index
	return index_id;
}
int main(int argc,char* argv[])
{
	ros::init(argc, argv, "nut_navigation_node");
  ros::NodeHandle nh;
	//Param
	const ros::Rate control_rate(ros::param::param("~control_rate", 1.));
	//Publisher
	g_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
	//Subscriber
	ros::Subscriber sub_path = nh.subscribe("path",1000,pathDataCallback);
	tf2_ros::TransformListener tf_sub(tf_buf, nh);
	//Timer
	ros::Timer timer = nh.createTimer(control_rate.expectedCycleTime(), timerCallback);

	ros::spin();
}
