#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <DynamicsProject/GraspFridge.h>
#include <DynamicsProject/ReleaseFridge.h>
#include <stdlib.h>

#define ENCODING "rgb8"
#define R_LOW 110
#define R_HIGH 130
#define G_LOW 1
#define G_HIGH 21
#define B_LOW 69
#define B_HIGH 89
#define CLUSTER_RADIUS 20

class FridgeDetector
{
public:
	struct pointCluster{
		int numPoints;
		int xCenter;
		int yCenter;
	};
	
	std::vector<pointCluster> pointClusters;
	
	ros::NodeHandle n;
	
	ros::ServiceServer graspFridgeServer;
	ros::ServiceServer releaseFridgeServer;
	ros::Subscriber imageSubscriber;
	
	cv_bridge::CvImagePtr img;
	
	FridgeDetector();
	
	bool graspFridge(DynamicsProject::GraspFridge::Request &req, DynamicsProject::GraspFridge::Response &res);
	
	bool releaseFridge(DynamicsProject::ReleaseFridge::Request &req, DynamicsProject::ReleaseFridge::Response &res);
	
	void imageCallback(const sensor_msgs::Image& img_msg);
};
