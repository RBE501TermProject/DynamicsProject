#include "DynamicsProject/FridgeDetector.h"


using namespace std;
using namespace cv;
using namespace pcl;

FridgeDetector::FridgeDetector() :
	acMoveRightArm("move_right_arm", true),
	acMoveLeftArm("move_left_arm", true),
	acLeftGripper("/l_gripper_controller/gripper_action", true),
	acRightGripper("/r_gripper_controller/gripper_action", true)
{
	frontPressure = 0;
	
	ROS_INFO("Waiting for move_right_arm action server...");
	acMoveRightArm.waitForServer();
	ROS_INFO("Connected to move_right_arm server");
	
	ROS_INFO("Waiting for move_left_arm action server...");
	acMoveRightArm.waitForServer();
	ROS_INFO("Connected to move_left_arm server");
	
	ROS_INFO("Waiting for right gripper action server...");
	acRightGripper.waitForServer();
	ROS_INFO("Finsihed waiting for right gripper action server");
	
	ROS_INFO("Waiting for left gripper action server...");
	acLeftGripper.waitForServer();
	ROS_INFO("Finsihed waiting for left gripper action server");

	graspFridgeServer = n.advertiseService("fridge_detector/grasp_fridge", &FridgeDetector::graspFridge, this);
	releaseFridgeServer = n.advertiseService("fridge_detector/release_fridge", &FridgeDetector::releaseFridge, this);
	
	baseVelPublisher = n.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
	basePathPublisher = n.advertise<nav_msgs::Path>("base_controller/path", 1);
	imageSubscriber = n.subscribe("/head_mount_kinect/rgb/image_color", 1, &FridgeDetector::imageCallback, this);
	cloudSubscriber = n.subscribe<sensor_msgs::PointCloud2>("/head_mount_kinect/depth_registered/points", 1, &FridgeDetector::cloudCallback, this);
	rGripperPressureSubscriber = n.subscribe("/pressure/r_gripper_motor", 1, &FridgeDetector::rGripperPressureCallback, this);
}

void FridgeDetector::rGripperPressureCallback(pr2_msgs::PressureState gripperPressure)
{
	rGripperPressure = gripperPressure;
	
	frontPressure = (int)((rGripperPressure.r_finger_tip[3] + rGripperPressure.r_finger_tip[4]) / 2);
}

bool FridgeDetector::graspFridge(DynamicsProject::GraspFridge::Request &req, DynamicsProject::GraspFridge::Response &res)
{
	openFridge();

	if(img == NULL) {
		ROS_ERROR("No image has been recieved");
		return false;
	}

	blur(img->image, img->image, Size(3, 3));
	int numBytes = img->image.step * img->image.rows;

	for (int pixelItt = 0; pixelItt < numBytes; pixelItt += 3)
	{
		int r = img->image.data[pixelItt];
		int g = img->image.data[pixelItt+1];
		int b = img->image.data[pixelItt+2];
		
		if (r > R_LOW && r < R_HIGH && g > G_LOW && g < G_HIGH && b > B_LOW && b < B_HIGH)
		{	
			int pointX = (int)(pixelItt/(img->image.step));
          	int pointY = (int)(pixelItt - pointX * (img->image.step));
          	pointY = (int)(pointY/3);
          	if (pointClusters.size() == 0)
          	{
          		pointCluster *temp = new pointCluster();
          		temp->numPoints = 1;
          		temp->xCenter = pointX;
          		temp->yCenter = pointY;
          		pointClusters.push_back(*(new pointCluster()));
          		pointClusters[pointClusters.size() - 1] = *temp;
          		delete temp;
          	}
          	else
          	{
          		bool added = false;
          		for (unsigned int i = 0; i < pointClusters.size(); i++)
          		{
          			if (abs(pointX - pointClusters[i].xCenter) < CLUSTER_RADIUS && abs(pointY - pointClusters[i].yCenter) < CLUSTER_RADIUS)
          			{
          				pointClusters[i].xCenter = (pointClusters[i].xCenter * pointClusters[i].numPoints + pointX)/(pointClusters[i].numPoints + 1);
          				pointClusters[i].yCenter = (pointClusters[i].yCenter * pointClusters[i].numPoints + pointY)/(pointClusters[i].numPoints + 1);
          				pointClusters[i].numPoints ++;
          				added = true;
          				break;
          			}
          		}
          		if (!added)
      			{
      				pointCluster *temp2 = new pointCluster();
			  		temp2->numPoints = 1;
			  		temp2->xCenter = pointX;
			  		temp2->yCenter = pointY;
			  		pointClusters.push_back(*(new pointCluster()));
			  		pointClusters[pointClusters.size() - 1] = *temp2;
			  		delete temp2;
      			}
          	}
          	
		}
	}
	ROS_INFO("Number of clusters: %d", pointClusters.size());
	if (pointClusters.size() > 0)
	{
		int maxPoints = 0;
		int maxIndex = 0;
		for (unsigned int i = 0; i < pointClusters.size(); i ++)
		{
			if (pointClusters[i].numPoints > maxPoints)
			{
				maxPoints = pointClusters[i].numPoints;
				maxIndex = i;
			}
		}
		ROS_INFO("Cluster center: (%d, %d)", pointClusters[maxIndex].xCenter, pointClusters[maxIndex].yCenter);
		
		ROS_INFO("width:%d, height:%d", latest_cloud.width, latest_cloud.height);
		
		PointXYZ p = latest_cloud.at(pointClusters[maxIndex].yCenter, pointClusters[maxIndex].xCenter);
		
		geometry_msgs::PointStamped point;
		point.header = cloud->header;
		point.point.x = p.x;
		point.point.y = p.y;
		point.point.z = p.z;
		
		geometry_msgs::PointStamped fridgePoint;
		
		listener.transformPoint("/base_link", point, fridgePoint);
		//ROS_INFO("Point: (%f, %f, %f)", fridgePoint.point.x, fridgePoint.point.y, fridgePoint.point.z);
		
		pr2_controllers_msgs::Pr2GripperCommandGoal openGripper;
		openGripper.command.position = 0.08;
		openGripper.command.max_effort = -1.0;
		acRightGripper.sendGoal(openGripper);
		acRightGripper.waitForResult(ros::Duration(5));
		
		arm_navigation_msgs::MoveArmGoal armGoal;
		armGoal.motion_plan_request.group_name = "right_arm";
		armGoal.motion_plan_request.num_planning_attempts = 1;
		armGoal.motion_plan_request.planner_id = string("");
		armGoal.planner_service_name = string("ompl_planning/plan_kinematic_path");
		armGoal.motion_plan_request.allowed_planning_time = ros::Duration(10.0);
		armGoal.accept_partial_plans = true;
		armGoal.accept_invalid_goals = true;
		armGoal.disable_collision_monitoring = true;
	
		arm_navigation_msgs::SimplePoseConstraint goalPose;
		goalPose.header.frame_id = "base_link";
		goalPose.link_name = "r_wrist_roll_link";
		goalPose.pose.position.x = fridgePoint.point.x-.22;
		goalPose.pose.position.y = fridgePoint.point.y;
		goalPose.pose.position.z = fridgePoint.point.z;
		
		goalPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(3.14159/2.0, 0.0, 0.0);
	
		goalPose.absolute_position_tolerance.x = .02;
		goalPose.absolute_position_tolerance.y = .02;
		goalPose.absolute_position_tolerance.z = .02;
		goalPose.absolute_roll_tolerance = .2;
		goalPose.absolute_pitch_tolerance = .2;
		goalPose.absolute_yaw_tolerance = .2;
	
		arm_navigation_msgs::addGoalConstraintToMoveArmGoal(goalPose, armGoal);
	
		acMoveRightArm.sendGoal(armGoal);
		acMoveRightArm.waitForResult(ros::Duration(10.0));
		actionlib::SimpleClientGoalState state = acMoveRightArm.getState();
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Action finished: %s", state.toString().c_str());
			
			geometry_msgs::Twist vel;
			vel.linear.x = .2;
			
			ros::Rate loopRate(60);
			
			while(frontPressure < 3000)
			{
				baseVelPublisher.publish(vel);
				loopRate.sleep();
				ros::spinOnce();
			}
			
			geometry_msgs::Twist stopVel;
			stopVel.linear.x = 0;
			baseVelPublisher.publish(stopVel);
			
			pr2_controllers_msgs::Pr2GripperCommandGoal closeGripper;
			closeGripper.command.position = 0.0;
			closeGripper.command.max_effort = -1.0;
			acRightGripper.sendGoal(closeGripper);
			acRightGripper.waitForResult(ros::Duration(5));
			
			ROS_INFO("Opening fridge?");
			openFridge();
		}
		else
		{
			ROS_INFO("Action failed: %s", state.toString().c_str());
		}
	}
	
	
	pointClusters.clear();
	return true;
}

bool FridgeDetector::releaseFridge(DynamicsProject::ReleaseFridge::Request &req, DynamicsProject::ReleaseFridge::Response &res)
{
	
	return true;
}

void FridgeDetector::imageCallback(const sensor_msgs::Image& img_msg)
{
	img = cv_bridge::toCvCopy(img_msg, ENCODING);
	img->header.seq = img_msg.header.seq;
	img->header.stamp = ros::Time::now();
	img->header.frame_id = img_msg.header.frame_id;
}

void FridgeDetector::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
	fromROSMsg(*pc, latest_cloud);
	cloud = pc;
}

void FridgeDetector::openFridge() {
	nav_msgs::Path path;
	int numSteps = 15;
	ros::Duration dt(3.0 / ((double) numSteps));
	ros::Time t = ros::Time::now();
	for(int i=0; i<=numSteps; i++) {
		geometry_msgs::PoseStamped pose, odomPose;
		double theta = ((double) i) * M_PI_2 / ((double) numSteps);
		pose.pose.position.x = -sin(theta) * FRIDGE_RADIUS;
		pose.pose.position.y = cos(theta) * FRIDGE_RADIUS - FRIDGE_RADIUS;
		ROS_INFO_STREAM("Point: " << pose.pose.position.x << ", " << pose.pose.position.y);
		pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
		//pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/base_link";
		listener.transformPose("/odom_combined", pose, odomPose);
		ROS_INFO_STREAM("Odom Point: " << odomPose.pose.position.x << ", " << odomPose.pose.position.y);
		t += dt;
		odomPose.header.stamp = t;
		path.poses.push_back(odomPose);
	}
	
	basePathPublisher.publish(path);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fridge_detector");
	
	FridgeDetector fd;
	
	ros::spin();
	
	return 0;
}
