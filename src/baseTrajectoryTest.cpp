#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "base_trajectory_test");
	ros::NodeHandle n;

	ros::Publisher pathPub = n.advertise<nav_msgs::Path>("base_controller/path",
			10);

	while (ros::Time::now().sec == 0);

	nav_msgs::Path path;

	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 1.0;
	pose.pose.position.y = 0.5;
	tf::Quaternion quat = tf::createQuaternionFromYaw(angles::from_degrees(-45));
	tf::quaternionTFToMsg(quat, pose.pose.orientation);
	pose.header.stamp = ros::Time::now() + ros::Duration(3.0);
	path.poses.push_back(pose);

	pose.pose.position.x = 1.5;
	pose.pose.position.y = -1.0;
	quat = tf::createQuaternionFromYaw(angles::from_degrees(45));
	tf::quaternionTFToMsg(quat, pose.pose.orientation);
	pose.header.stamp = ros::Time::now() + ros::Duration(6.0);
	path.poses.push_back(pose);

	pose.pose.position.x = 0.0;
	pose.pose.position.y = 0.0;
	quat = tf::createQuaternionFromYaw(angles::from_degrees(0));
	tf::quaternionTFToMsg(quat, pose.pose.orientation);
	pose.header.stamp = ros::Time::now() + ros::Duration(10.0);
	path.poses.push_back(pose);

	pathPub.publish(path);
	ROS_INFO("Path published");

	ros::spinOnce();

	ros::Duration(1).sleep();

	return 0;
}

