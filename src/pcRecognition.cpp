#include "grasp_learning/pcRecognition.h"

using namespace std;
using namespace pcl;

pcRecognition::pcRecognition()
{
	xTrans = 0.0;
	yTrans = 0.0;
	zTrans = 0.0;

	readGraspClient = n.serviceClient<grasp_learning::ReadGrasp>("grasp_reader/read_grasps");
	recognizeAndGraspServer = n.advertiseService("grasp_learning/recognize_and_pickup", &pcRecognition::recognizeAndPickup, this);
	
	readPointClouds();
}

bool pcRecognition::recognizeAndPickup(grasp_learning::RecognizeAndGrasp::Request &req, grasp_learning::RecognizeAndGrasp::Response &res)
{
	PointCloud<PointXYZRGB>::Ptr baseCloudPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr targetCloudPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);
	
	vector<geometry_msgs::Pose> baseGraspList;
	vector<geometry_msgs::Pose> targetGraspList;
	vector<geometry_msgs::Pose> resultGraspList;

	//convert point cloud to pcl format
	sensor_msgs::PointCloud2 tempCloud;
	sensor_msgs::convertPointCloudToPointCloud2(req.cloud, tempCloud);
	fromROSMsg(tempCloud, *targetCloudPtr);
	
	//Filter input point cloud to remove noise, translate it to the origin for easier visualization
	filterCloudOutliers(targetCloudPtr, RADIUS, NUM_NEIGHBORS);
	translateToOrigin(targetCloudPtr, &targetGraspList, false);
	
	//Recognition
	float minScore = 999;
	int minIndex = 0;
	for (unsigned int j = 0; j < models.size(); j ++)
	{
		baseCloudPtr = models[j];

		float tempScore = scoreRegistration(baseCloudPtr, targetCloudPtr, false);
		if (tempScore < minScore)
		{
			minScore = tempScore;
			minIndex = j;
		}
	}
	
	ROS_INFO("Point cloud recognized as Model %d", minIndex + 1);
	
	//Determine possible grasps
	baseGraspList.clear();
	targetGraspList = graspLists[minIndex];
	vector<geometry_msgs::Pose> finalGraspList;
	icpRegistration(targetCloudPtr, models[minIndex], baseGraspList, targetGraspList, &finalGraspList, false, false);
	
	//TODO: Execute grasps using rail_grasping
	
	
	return true;
}

bool pcRecognition::getCloud(string filename, PointCloud<PointXYZRGB>::Ptr pointcloudOut, vector<geometry_msgs::Pose> *graspListOut)
{	
	grasp_learning::ReadGrasp srv;
	srv.request.grasp_entry = filename;

	readGraspClient.call(srv);

	if (srv.response.success)
	{	
		sensor_msgs::convertPointCloudToPointCloud2(srv.response.grasp.pointCloud, baseCloud);
		fromROSMsg(baseCloud, *pointcloudOut);
		
		*graspListOut = srv.response.grasp.gripperPoses;
	}

	return srv.response.success;	
}

void pcRecognition::readPointClouds()
{	
	ROS_INFO("Reading models...");
	bool reading = true;
	stringstream ss;
	int count = 1;
	while (reading)
	{
		/*
		ss.str("");
		ss << "model_" << count << ".pcd";
		PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
		if (io::loadPCDFile<PointXYZRGB>(ss.str(), *tempCloudPtr) == -1)
		{
			reading = false;
			count --;
		}
		else
		{
			models.push_back(tempCloudPtr);
			count ++;
		}
		*/
		
		ss.str("");
		ss << "model_" << count << ".txt";
		PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
		vector<geometry_msgs::Pose> tempGraspList;
		reading = getCloud(ss.str(), tempCloudPtr, &tempGraspList);
		if (reading)
		{
			models.push_back(tempCloudPtr);
			graspLists.push_back(tempGraspList);
			count ++;
		}
		else
			count --;
	}
	ROS_INFO("Read %d Models", count);
}

void pcRecognition::recognizePointClouds()
{
	/*
	vector<PointCloud<PointXYZRGB>::Ptr> models;
	vector<PointCloud<PointXYZRGB>::Ptr> pointClouds;
	*/

	/*
	PointCloud<PointXYZRGB>::Ptr baseCloudPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr targetCloudPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);
	*/
	
	/*
	ROS_INFO("Reading models...");
	bool reading = true;
	stringstream ss;
	int count = 1;
	while (reading)
	{
		ss.str("");
		ss << "model_" << count << ".pcd";
		PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
		if (io::loadPCDFile<PointXYZRGB>(ss.str(), *tempCloudPtr) == -1)
		{
			reading = false;
			count --;
		}
		else
		{
			models.push_back(tempCloudPtr);
			count ++;
		}
	}
	ROS_INFO("Read %d Models", count);
	
	ROS_INFO("Reading unrecognized point clouds...");
	reading = true;
	count = 1;
	while (reading)
	{
	*/
		/*
		//UNCOMMENT FOR USER STUDY DATA FORMAT
		ss.str("");
		ss << "grasp_" << count << ".txt";
		PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
		reading = getCloud(ss.str(), tempCloudPtr);
		if (reading)
		{
			pointClouds.push_back(tempCloudPtr);
			count ++;
		}
		else
			count --;
		*/
		
		/*
		//UNCOMMENT FOR PCL DATA FORMAT
		ss.str("");
		ss << "cloud_" << count << ".pcd";
		PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
		if (io::loadPCDFile<PointXYZRGB>(ss.str(), *tempCloudPtr) == -1)
		{
			reading = false;
			count --;
		}
		else
		{
			pointClouds.push_back(tempCloudPtr);
			count ++;
		}
	}
	ROS_INFO("Read %d Point Clouds", count);
	*/
	
	/*
	//Filter point clouds to remove noise, translate them to the origin for easier visualization
	for (unsigned int i = 0; i < pointClouds.size(); i ++)
	{
		filterCloudOutliers(pointClouds[i], RADIUS, NUM_NEIGHBORS);
		translateToOrigin(pointClouds[i], false);
	}
	
	//Recognition
	for (unsigned int i = 0; i < pointClouds.size(); i ++)
	{
		float minScore = 999;
		int minIndex = 0;
		for (unsigned int j = 0; j < models.size(); j ++)
		{
			baseCloudPtr = models[j];
			targetCloudPtr = pointClouds[i];

			float tempScore = scoreRegistration(baseCloudPtr, targetCloudPtr, false);
			if (tempScore < minScore)
			{
				minScore = tempScore;
				minIndex = j;
			}
		}
		
		ROS_INFO("Grasp %d recognized as Model %d", i + 1, minIndex + 1);
	}
	*/
}

float pcRecognition::scoreRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, bool debug)
{
	IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
	icp.setInputCloud(targetCloudPtr);
	icp.setInputTarget(baseCloudPtr);
	PointCloud<PointXYZRGB>::Ptr targetTransformedPtr(new PointCloud<PointXYZRGB>);
	icp.align(*targetTransformedPtr);
	float icpScore = icp.getFitnessScore();
	//ROS_INFO_STREAM("ICP convergence score: " << icpScore);
	
	float dstError = calculateRegistrationMetricDstError(baseCloudPtr, targetTransformedPtr);
	float colorError = calculateRegistrationMetricOverlap(baseCloudPtr, targetTransformedPtr, .005);
	//float avgColorDiff = calculateRegistrationMetricColorRange(baseCloudPtr, targetTransformedPtr);
	//float maxDstDiff = calculateRegistrationMetricDistance(baseCloudPtr, targetTransformedPtr);
	//ROS_INFO("Calculated distance error score: %f", dstError);
	//ROS_INFO("Calculated overlap score: %f", overlap);
	
	float result = ALPHA * (3*dstError) + (1 - ALPHA) * (colorError/100.0);
	
	if (debug)
	{
		visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(0, 0, 0);
		visualization::PointCloudColorHandlerRGBField<PointXYZRGB> colorHandler1(baseCloudPtr);
		visualization::PointCloudColorHandlerRGBField<PointXYZRGB> colorHandler2(targetTransformedPtr);
		viewer.addPointCloud(baseCloudPtr, colorHandler1, "base point cloud");
		viewer.addPointCloud(targetTransformedPtr, colorHandler2, "target point cloud");
		
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			pcl_sleep(0.01);
		}
	}
	
	return result;
}

PointCloud<PointXYZRGB>::Ptr pcRecognition::icpRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, vector<geometry_msgs::Pose> baseGrasps, vector<geometry_msgs::Pose> targetGrasps, vector<geometry_msgs::Pose> *resultGrasps, bool scoreFiltered, bool debug)
{
	IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
	icp.setInputCloud(targetCloudPtr);
	icp.setInputTarget(baseCloudPtr);
	PointCloud<PointXYZRGB>::Ptr targetTransformedPtr(new PointCloud<PointXYZRGB>);
	icp.align(*targetTransformedPtr);
	float icpScore = icp.getFitnessScore();
	//ROS_INFO_STREAM("ICP convergence score: " << icpScore);
	
	//float dstError = calculateRegistrationMetricDstError(baseCloudPtr, targetTransformedPtr);
	//float overlap = calculateRegistrationMetricOverlap(baseCloudPtr, targetTransformedPtr, .005);
	//float avgColorDiff = calculateRegistrationMetricColorRange(baseCloudPtr, targetTransformedPtr);
	//float maxDstDiff = calculateRegistrationMetricDistance(baseCloudPtr, targetTransformedPtr);
	//ROS_INFO("Calculated distance error score: %f", dstError);
	//ROS_INFO("Calculated overlap score: %f", overlap);
	
	PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>& result = *resultPtr;
	
	if (scoreFiltered)
	{
		if (icpScore > .00004)
		{
			return resultPtr;
		}
		else
		{
			result = *baseCloudPtr + *targetTransformedPtr;
	
			filterRedundentPoints(resultPtr, DST_THRESHOLD);
		}
	}
	else
	{
		//Transform grasps to the target cloud position and orientation
		Eigen::Matrix4f transform = icp.getFinalTransformation();
		tf::Matrix3x3 rotationMatrix(	transform(0,0), transform(0,1), transform(0,2),
									transform(1,0), transform(1,1), transform(1,2),
									transform(2,0), transform(2,1), transform(2,2));
		tf::Transform tfTransform;
		tf::Quaternion quat;
		rotationMatrix.getRotation(quat);
		tfTransform.setOrigin(tf::Vector3(transform(0,3), transform(1,3), transform(2,3)));
		tfTransform.setRotation(quat);
		
		ros::Time now = ros::Time::now();
		tfBroadcaster.sendTransform(tf::StampedTransform(tfTransform, now, "target_cloud_frame", "base_cloud_frame"));
		tfListener.waitForTransform("target_cloud_frame", "base_cloud_frame", now, ros::Duration(5.0));
		for (unsigned int i = 0; i < targetGrasps.size(); i ++)
		{
			geometry_msgs::PoseStamped poseOut;
			geometry_msgs::PoseStamped tempPoseStamped;
			tempPoseStamped.pose = targetGrasps[i];
			tempPoseStamped.header.stamp = now;
			tempPoseStamped.header.frame_id = "target_cloud_frame";
			
			tfListener.transformPose("base_cloud_frame", tempPoseStamped, poseOut);
			
			//undo origin translation
			poseOut.pose.position.x += xTrans;
			poseOut.pose.position.y += yTrans;
			poseOut.pose.position.z += zTrans;
			
			targetGrasps[i] = poseOut.pose;
		}
		
		//merge point clouds
		result = *baseCloudPtr + *targetTransformedPtr;
		
		//merge grasp lists
		for (unsigned int i = 0 ; i < baseGrasps.size(); i ++)
		{
			(*resultGrasps).push_back(baseGrasps[i]);
		}
		for (unsigned int i = 0; i < targetGrasps.size(); i ++)
		{
			(*resultGrasps).push_back(targetGrasps[i]);
		}
	
		filterRedundentPoints(resultPtr, DST_THRESHOLD);
	}
	
	//classifyMerge(overlap, maxDstDiff, dstError, avgColorDiff);
	
	if (debug)
	{
		visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(0, 0, 0);
		visualization::PointCloudColorHandlerRGBField<PointXYZRGB> colorHandler1(baseCloudPtr);
		visualization::PointCloudColorHandlerRGBField<PointXYZRGB> colorHandler2(targetTransformedPtr);
		viewer.addPointCloud(baseCloudPtr, colorHandler1, "base point cloud");
		viewer.addPointCloud(targetTransformedPtr, colorHandler2, "target point cloud");
		
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			pcl_sleep(0.01);
		}
	}
	
	//return targetTransformedPtr;
	return resultPtr;
}


float pcRecognition::calculateRegistrationMetricDstError(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{	
	float score = 0;
	KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
	searchTree.setInputCloud(baseCloudPtr);
	vector<int> removeIndices;
	vector<int> indices;
	vector<float> distances;
	
	for (unsigned int i = 0; i < targetCloudPtr->size(); i ++)
	{
		searchTree.nearestKSearch(targetCloudPtr->at(i), 1, indices, distances);
		score += distances[0];
	}
	
	return score;
}

float pcRecognition::calculateRegistrationMetricOverlap(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, float dstThreshold)
{
	float score = 0;
	float colorError = 0;
	KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
	searchTree.setInputCloud(baseCloudPtr);
	vector<int> removeIndices;
	vector<int> indices;
	vector<float> distances;
	
	for (unsigned int i = 0; i < targetCloudPtr->size(); i ++)
	{
		PointXYZRGB searchPoint = targetCloudPtr->at(i);
		int neighbors = searchTree.radiusSearch(searchPoint, dstThreshold, indices, distances);
		if (neighbors > 0)
		{
			score ++;
			
			float colorDistance = 0;
			for (unsigned int j = 0; j < indices.size(); j ++)
			{
				PointXYZRGB point = baseCloudPtr->at(indices[j]);
				colorDistance += sqrt(pow(searchPoint.r - point.r, 2) + pow(searchPoint.g - point.g, 2) + pow(searchPoint.b - point.b, 2));
			}
			colorDistance /= neighbors;
			colorError += colorDistance;
		}
	}
	
	colorError /= score;	
	score /= targetCloudPtr->size();
	
	//ROS_INFO("Color Error: %f", colorError);
	//ROS_INFO("Overlap Score: %f", score);

	return colorError;
}

float pcRecognition::calculateRegistrationMetricColorRange(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{
	float avgr = 0, avgg = 0, avgb = 0;
	
	for (unsigned int i = 0; i < targetCloudPtr->size(); i ++)
	{
		PointXYZRGB point = targetCloudPtr->at(i);
			
		avgr += point.r;
		avgg += point.g;
		avgb += point.b;
	}
	
	//ROS_INFO("Target AVG Red: %f", avgr /= targetCloudPtr->size());
	//ROS_INFO("Target AVG Green: %f", avgg /= targetCloudPtr->size());
	//ROS_INFO("Target AVG Blue: %f", avgb /= targetCloudPtr->size());
	
	float avg1 = (avgr + avgg + avgb) / targetCloudPtr->size();
	
	avgr = 0;
	avgg = 0;
	avgb = 0;
		
	for (unsigned int i = 0; i < baseCloudPtr->size(); i ++)
	{
		PointXYZRGB point = baseCloudPtr->at(i);
			
		avgr += point.r;
		avgg += point.g;
		avgb += point.b;
	}
	
	//ROS_INFO("Base AVG Red: %f", avgr /= targetCloudPtr->size());
	//ROS_INFO("Base AVG Green: %f", avgg /= targetCloudPtr->size());
	//ROS_INFO("Base AVG Blue: %f", avgb /= targetCloudPtr->size());
	
	float avg2 = (avgr + avgg + avgb) / baseCloudPtr->size();
	
	return fabs(avg1 - avg2);
}

float pcRecognition::calculateRegistrationMetricDistance(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{
	float maxDst = 0;
	
	for (unsigned int i = 0; i < targetCloudPtr->size() - 1; i ++)
	{
		for (unsigned int j = 1; j < targetCloudPtr->size(); j ++)
		{
			PointXYZRGB p1 = targetCloudPtr->at(i);
			PointXYZRGB p2 = targetCloudPtr->at(j);
			
			float dst = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
			if (dst > maxDst)
				maxDst = dst;
		}
	}
	
	//ROS_INFO("Max distance for target: %f", maxDst);	
	
	float maxDst2 = 0;
	
	for (unsigned int i = 0; i < baseCloudPtr->size() - 1; i ++)
	{
		for (unsigned int j = 1; j < baseCloudPtr->size(); j ++)
		{
			PointXYZRGB p1 = baseCloudPtr->at(i);
			PointXYZRGB p2 = baseCloudPtr->at(j);
			
			float dst = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
			if (dst > maxDst2)
				maxDst2 = dst;
		}
	}
	
	//ROS_INFO("Max distance for base: %f", maxDst3);
	
	return fabs(maxDst - maxDst2);
}

void pcRecognition::filterCloudOutliers(PointCloud<PointXYZRGB>::Ptr cloudPtr, double radius, int numNeighborThreshold, bool debug)
{
	KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
	searchTree.setInputCloud(cloudPtr);
	vector<int> removeIndices;
	vector<int> indices;
	vector<float> distances;
	
	for (unsigned int i = 0; i < cloudPtr->size(); i ++)
	{
		int neighbors = searchTree.radiusSearch(cloudPtr->at(i), radius, indices, distances);
		if (neighbors < numNeighborThreshold)
			removeIndices.push_back(i);
	}
	
	sort(removeIndices.begin(), removeIndices.end());
	reverse(removeIndices.begin(), removeIndices.end());
	
	ROS_INFO("Found %d points to filter", removeIndices.size());
	
	for (int i = (int)(removeIndices.size()) - 1; i >= 0; i --)
	{
		cloudPtr->erase(cloudPtr->begin() + i);
	}
	
	
	if (debug)
	{
		visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(0, 0, 0);
		visualization::PointCloudColorHandlerRGBField<PointXYZRGB> colorHandler1(cloudPtr);
		viewer.addPointCloud(cloudPtr, colorHandler1, "filtered point cloud");
		
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			pcl_sleep(0.01);
		}
	}
}

void pcRecognition::filterRedundentPoints(PointCloud<PointXYZRGB>::Ptr cloudPtr, double dstThreshold)
{
	KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
	searchTree.setInputCloud(cloudPtr);
	vector<int> removeIndices;
	vector<int> indices;
	vector<float> distances;
	
	for (int i = (int)(cloudPtr->size()) - 1; i >= 0; i --)
	{
		int neighbors = searchTree.radiusSearch(cloudPtr->at(i), dstThreshold, indices, distances);
		if (neighbors > 1)
			cloudPtr->erase(cloudPtr->begin() + i);
	}
}

void pcRecognition::translateToOrigin(PointCloud<PointXYZRGB>::Ptr cloudPtr, vector<geometry_msgs::Pose> *grasps, bool debug)
{
	float x = 0;
	float y = 0;
	float z = 0;
	
	for (unsigned int i = 0; i < cloudPtr->size(); i ++)
	{
		x += cloudPtr->at(i).x;
		y += cloudPtr->at(i).y;
		z += cloudPtr->at(i).z;
	}
	x /= cloudPtr->size();
	y /= cloudPtr->size();
	z /= cloudPtr->size();
	
	Eigen::Matrix4f transform;
	transform << 1, 0, 0, -x,
				 0, 1, 0, -y,
				 0, 0, 1, -z,
				 0, 0, 0, 1;
	
	transformPointCloud(*cloudPtr, *cloudPtr, transform);
		
	//transform grasps
	xTrans = x;
	yTrans = y;
	zTrans = z;
	for (unsigned int i = 0; i < (*grasps).size(); i ++)
	{
		(*grasps)[i].position.x -= x;
		(*grasps)[i].position.y -= y;
		(*grasps)[i].position.z -= z;
	}
	
	if (debug)
	{
		visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(0, 0, 0);
		visualization::PointCloudColorHandlerRGBField<PointXYZRGB> colorHandler1(cloudPtr);
		viewer.addPointCloud(cloudPtr, colorHandler1, "translated point cloud");
		
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			pcl_sleep(0.01);
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pc_recognition");
	
	pcRecognition pcr;
	
	pcr.recognizePointClouds();
	
	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
