#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <grasp_learning/ReadGrasp.h>
#include <stdlib.h>
#include <pr2_msgs/PressureState.h>
#include <iostream>
#include <fstream>
#include <sstream>

class readGrasps
{
public:			
	//ROS publishers, subscribers, and action servers
	ros::NodeHandle n;
	
	ros::ServiceServer readGraspServer;
	
	/**
	 * Constructor
	 */
	readGrasps();
	
	/**
	 * Service callback for grasp reading
	 * @param req service request
	 * @param res service response
	 * @return true on success
	 */
	bool readGraspService(grasp_learning::ReadGrasp::Request &req, grasp_learning::ReadGrasp::Response &res);
};
