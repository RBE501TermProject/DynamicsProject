#include "grasp_learning/pcSaver.h"

using namespace std;
using namespace pcl;

pcSaver::pcSaver()
{
	count = 1;

	cloudPublisher = n.advertise<sensor_msgs::PointCloud>("pc_saver/cloud", 1);
	segmentationClient = n.serviceClient<tabletop_object_detector::TabletopDetection>("/object_detection");
	segmentationServer = n.advertiseService("pc_saver/segment", &pcSaver::segmentService, this);
}

bool pcSaver::segmentService(grasp_learning::Segment::Request &req, grasp_learning::Segment::Response &res)
{
	tabletop_object_detector::TabletopDetection srv;
	srv.request.return_clusters = true;
	srv.request.return_models = true;
	srv.request.num_models = 1;
	
	if (!segmentationClient.call(srv))
	{
		ROS_INFO("Error calling segmentation.");
		res.success = false;
		return false;
	}
	if (srv.response.detection.result != srv.response.detection.SUCCESS)
	{
		ROS_INFO("Tabletop detection returned error code %d", srv.response.detection.result);
		res.success = false;
		return true;
	}
	if (srv.response.detection.clusters.empty())
	{
		ROS_INFO("No objects were found.");
		res.success = false;
		return true;
	}
	
	res.segmentedClouds = srv.response.detection.clusters;
	res.success = true;
	
	//save point clouds
	clouds = srv.response.detection.clusters;
	for (unsigned int i = 0; i < clouds.size(); i ++)
	{
		ROS_INFO("Saving point cloud %d...", count);
		
		//convert to pcl point cloud
		sensor_msgs::PointCloud2 tempCloud;
		PointCloud<PointXYZRGB>::Ptr pclCloudPtr(new PointCloud<PointXYZRGB>);
		sensor_msgs::convertPointCloudToPointCloud2(clouds[i], tempCloud);
		fromROSMsg(tempCloud, *pclCloudPtr);
		
		//save as pcd file
		stringstream ss;
		ss.str("");
		ss << "cloud_" << count << ".pcd";
		io::savePCDFileASCII(ss.str(), *pclCloudPtr);
		
		count ++;
	}
	
	ROS_INFO("Finished.");
	return true;
}

void pcSaver::publishClouds()
{
	//publish the first point cloud
	if (clouds.size() > 0)
	{
		cloudPublisher.publish(clouds[0]);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pc_registration");
	
	pcSaver pcs;
	
	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		ros::spinOnce();
		pcs.publishClouds();
		loop_rate.sleep();
	}
	
	return 0;
}
