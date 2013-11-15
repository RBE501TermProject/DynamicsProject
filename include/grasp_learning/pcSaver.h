#include <ros/ros.h>

#include <grasp_learning/Segment.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tabletop_object_detector/TabletopDetection.h>

#include <boost/thread/thread.hpp>
#include <stdlib.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

class pcSaver
{
public:
	int count;

	//ROS publishers, subscribers, and action servers
	ros::NodeHandle n;
	
	ros::Publisher cloudPublisher;
	ros::ServiceClient segmentationClient;
	ros::ServiceServer segmentationServer;
	
	//Point clouds
	std::vector<sensor_msgs::PointCloud> clouds;
	
	/**
	 * Constructor
	 */
	pcSaver();
	
	/**
	 * Callback for the point cloud segmentation
	 */
	bool segmentService(grasp_learning::Segment::Request &req, grasp_learning::Segment::Response &res);
	
	/**
	 * Point cloud publishing for ROS visualization
	 */
	void publishClouds();
	
	bool classifyMerge(float overlap, float maxDstDiff, float dstError, float avgColorDiff);
};
