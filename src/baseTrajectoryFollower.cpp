/**
 * @file baseTrajectoryFollower.cpp
 * @brief Follows a trajectory for the PR2's base using cubic polynomial trajectories
 * @author Tim Jenkel
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <Eigen/Dense>

/// Publisher for base velocity commands
ros::Publisher baseCommandPub;

/// Last odometry message recieved
nav_msgs::OdometryConstPtr lastOdom;

/**
 * Calculates the cubic trajectory coefficients for the trajectory
 * between the current position and the goal point.
 * @param path The path that should be followed
 * @param i The index of the next point on the path
 * @return A matrix of the cubic trajectory coefficients for x, y, and theta
 */
Eigen::Matrix<double, 4, 3> cubicTrajCoeff(const nav_msgs::Path::ConstPtr& path,
		unsigned int i) {
	geometry_msgs::PoseStamped goal = path->poses[i];
	nav_msgs::OdometryConstPtr odom = lastOdom;
	double t0 = odom->header.stamp.toSec();
	double tf = goal.header.stamp.toSec();

	Eigen::Matrix4d T;
	T << 1, t0, pow(t0, 2), pow(t0, 3), 0, 1, 2 * t0, 3 * pow(t0, 2), 1, tf, pow(
			tf, 2), pow(tf, 3), 0, 1, 2 * tf, 3 * pow(tf, 2);

//	ROS_INFO_STREAM("T = " << std::endl << T);
	Eigen::Matrix<double, 4, 3> Q;
	double x0 = odom->pose.pose.position.x;
	double y0 = odom->pose.pose.position.y;

	tf::Quaternion odomQuat;
	tf::quaternionMsgToTF(odom->pose.pose.orientation, odomQuat);
	double th0 = tf::getYaw(odomQuat);
	tf::Vector3 odomVel;
	tf::vector3MsgToTF(odom->twist.twist.linear, odomVel);
	tf::Vector3 V0 = tf::quatRotate(odomQuat, odomVel);
	double dx0 = V0.x();
	double dy0 = V0.y();
	double dth0 = odom->twist.twist.angular.z;

	double xf = goal.pose.position.x;
	double yf = goal.pose.position.y;
	double thf = tf::getYaw(goal.pose.orientation);

	double dxf = 0, dyf = 0, dthf = 0;
	if (i + 1 < path->poses.size()) {
		geometry_msgs::PoseStamped nextGoal = path->poses[i + 1];
		double dt = nextGoal.header.stamp.toSec() - t0;
		dxf = (nextGoal.pose.position.x - x0) / dt;
		dyf = (nextGoal.pose.position.y - y0) / dt;
		dthf = (tf::getYaw(nextGoal.pose.orientation) - th0) / dt;
	}

	Q << x0, y0, th0, dx0, dy0, dth0, xf, yf, thf, dxf, dyf, dthf;

//	ROS_INFO_STREAM("Q = " << std::endl << Q);

	Eigen::Matrix<double, 4, 3> A;
	A = T.inverse() * Q;
//	ROS_INFO_STREAM("A = " << std::endl << A);
	return A;
}

/**
 * Calculates the error between the current position and the goal pose
 * @param goal The goal pose
 * @return The error transformation
 */
tf::Pose getErrorTF(geometry_msgs::PoseStamped goal) {
	tf::Pose goalTF, odomTF;
	tf::poseMsgToTF(goal.pose, goalTF);

	ros::spinOnce();
	if (lastOdom == NULL) {
		ROS_WARN("No odometry has been received");
		return goalTF;
	}

	tf::poseMsgToTF(lastOdom->pose.pose, odomTF);
	tf::Pose error = odomTF.inverseTimes(goalTF);
	ROS_INFO_STREAM(
			"Error: " << error.getOrigin().getX() << ", " << error.getOrigin().getY() << ", " << angles::to_degrees(tf::getYaw(error.getRotation())));
	return error;
}

/**
 * Calculates the point along a cubic trajectory at the given time
 * @param coeff The cubic trajectory coeffiecient matrix
 * @param time The time for the point
 */
geometry_msgs::PoseStamped getPoint(Eigen::Matrix<double, 4, 3> coeff,
		ros::Time time) {
	double t = time.toSec();
	Eigen::RowVector4d T;
	T << 1, t, pow(t, 2), pow(t, 3);
	Eigen::RowVector3d Q;
	Q = T * coeff;

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = time;
	pose.header.frame_id = lastOdom->header.frame_id;
	pose.pose.position.x = Q(0);
	pose.pose.position.y = Q(1);
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(Q(2)),
			pose.pose.orientation);
	return pose;
}

/**
 * Calculates the twist required to move the robot to the given point for its stamped time
 * @param The goal pose
 */
geometry_msgs::Twist getTwist(geometry_msgs::PoseStamped goal) {
	tf::Pose error = getErrorTF(goal);
	double dt = (goal.header.stamp - /*ros::Time::now()*/ lastOdom->header.stamp).toSec();
	ROS_INFO_STREAM("dt = " << dt);
	geometry_msgs::Twist twist;
	twist.linear.x = error.getOrigin().getX() / dt;
	twist.linear.y = error.getOrigin().getY() / dt;
	double yaw = tf::getYaw(error.getRotation());
	while (yaw > M_PI)
		yaw -= M_2_PI;
	while (yaw < -M_PI)
		yaw += M_2_PI;
	twist.angular.z = yaw / dt;
	ROS_INFO_STREAM(
			"twist: x=" << twist.linear.x << ", y=" << twist.linear.y << ", w=" << twist.angular.z);
	return twist;
}

/**
 * Follows the path
 * @param The path to follow
 */
void pathCallback(const nav_msgs::Path::ConstPtr& path) {
	ROS_INFO("path received");
	ros::Rate rate(20);
	for (unsigned int i = 0; i < path->poses.size(); i++) {
		ros::spinOnce();
		geometry_msgs::PoseStamped goal = path->poses[i];
		ROS_INFO_STREAM(
				"going to pose " << i << " (" << goal.pose.position.x << ", " << goal.pose.position.y << ")");
		Eigen::Matrix<double, 4, 3> coeff = cubicTrajCoeff(path, i);
		while (ros::ok() && ros::Time::now() + rate.expectedCycleTime() < goal.header.stamp) {
			ros::spinOnce();
			geometry_msgs::PoseStamped nextPoint;
//			if (dt < rate.cycleTime().toSec())
				nextPoint = goal;
//			else
//				nextPoint = getPoint(coeff,
//						ros::Time::now() + rate.expectedCycleTime());
			ROS_INFO_STREAM(
					"next point: (" << nextPoint.pose.position.x << ", " << nextPoint.pose.position.y << ")");
			geometry_msgs::Twist twist = getTwist(nextPoint);
			ROS_INFO_STREAM(
					"Publishing command: x=" << twist.linear.x << ", y=" << twist.linear.y << ", w=" << twist.angular.z);
			baseCommandPub.publish(twist);

			rate.sleep();
		}
	}
}

/**
 * The callback for odometry messages
 * @param The odometry message
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
	ROS_DEBUG_STREAM(
			"odometry received: " << odom->pose.pose.position.x << ", " << odom->pose.pose.position.y);
	lastOdom = odom;
}

/**
 * Initializes the base trajectory follower node and its subscribers and publishers
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "base_trajectory_follower");
	ros::NodeHandle n;

	baseCommandPub = n.advertise<geometry_msgs::Twist>(
			"base_controller/command", 10);
	ros::Subscriber pathSub = n.subscribe("base_controller/path", 1,
			pathCallback);
	ros::Subscriber odomSub = n.subscribe("base_odometry/odom", 5,
			odomCallback);

	ROS_INFO("base_trajectory_follower intialized");

	ros::spin();
	return 0;
}
