#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <DynamicsProject/GraspFridge.h>
#include <DynamicsProject/ReleaseFridge.h>
#include <stdlib.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_msgs/PressureState.h>
#include <nav_msgs/Path.h>

#define ENCODING "rgb8"
#define R_LOW 233
#define R_HIGH 253
#define G_LOW 20
#define G_HIGH 30
#define B_LOW 160
#define B_HIGH 180
#define CLUSTER_RADIUS 20
#define FRIDGE_RADIUS 0.42

class FridgeDetector
{
public:
	struct pointCluster{
		int numPoints;
		int xCenter;
		int yCenter;
	};
	
	std::vector<pointCluster> pointClusters;
	
	pcl::PointCloud<pcl::PointXYZ> latest_cloud;
	sensor_msgs::PointCloud2::ConstPtr cloud;
	
	tf::TransformListener listener;
	
	pr2_msgs::PressureState rGripperPressure;
	int frontPressure;
	
	ros::NodeHandle n;
	
	ros::ServiceServer graspFridgeServer;
	ros::ServiceServer releaseFridgeServer;
	ros::Publisher baseVelPublisher;
	ros::Publisher basePathPublisher;
	ros::Subscriber imageSubscriber;
	ros::Subscriber cloudSubscriber;
	ros::Subscriber rGripperPressureSubscriber;
	
	actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> acMoveRightArm;
	actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> acMoveLeftArm;
	actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> acLeftGripper;
	actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> acRightGripper;
	
	cv_bridge::CvImagePtr img;
	
	FridgeDetector();
	
	bool graspFridge(DynamicsProject::GraspFridge::Request &req, DynamicsProject::GraspFridge::Response &res);
	
	bool releaseFridge(DynamicsProject::ReleaseFridge::Request &req, DynamicsProject::ReleaseFridge::Response &res);
	
	void imageCallback(const sensor_msgs::Image& img_msg);
	
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc);
	
	void rGripperPressureCallback(pr2_msgs::PressureState gripperPressure);
	
	void openFridge();
};
