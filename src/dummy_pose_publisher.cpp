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
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf_utils.hpp"
// global variables
std::unique_ptr<tf2_ros::StaticTransformBroadcaster> g_tf_pub;
std::string g_parent_frame;
std::string g_child_frame;

class DummyBot{
public:
	DummyBot(){
	}
	~DummyBot(){
	}

};
void twistDataCallback(const geometry_msgs::Twist::ConstPtr twist){
	const ros::Time target_time = ros::Time::now();
	const double x =  
	const double y =
	const double z = 
	const geometry_msgs::TransformStamped tf = makeTransform(target_time,g_parent_frame,g_child_frame,x,y,z);
	
}

int main(int argc,char* argv[])
{	
	ros::init(argc, argv, "dummy_pose_publisher");
  ros::NodeHandle nh;
	//Publisher
	g_tf_pub.reset(new tf2_ros::StaticTransformBroadcaster());
	//Subscriber
	ros::Subscriber sub_twist = nh.subscribe("cmd_vel",1000,twistDataCallback);
	return 0;
}