#include "grasp_learning/pcRegistration.h"

using namespace std;
using namespace pcl;

pcRegistration::pcRegistration()
{
	baseCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("pc_registration/base_cloud", 1);
	targetCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("pc_registration/target_cloud", 1);
	readGraspClient = n.serviceClient<grasp_learning::ReadGrasp>("grasp_reader/read_grasps");
}

bool pcRegistration::getCloud(std::string filename, PointCloud<PointXYZRGB>::Ptr pointcloudOut, vector<geometry_msgs::Pose> *graspListOut)
{	
	grasp_learning::ReadGrasp srv;
	srv.request.grasp_entry = filename;

	readGraspClient.call(srv);

	if (srv.response.success)
	{	
		sensor_msgs::convertPointCloudToPointCloud2(srv.response.grasp.pointCloud, baseCloud);
		fromROSMsg(baseCloud, *pointcloudOut);
		baseCloudPublisher.publish(baseCloud);
		
		*graspListOut = srv.response.grasp.gripperPoses;
	}

	return srv.response.success;	
}

void pcRegistration::pairwiseRegisterPointclouds()
{
	vector<PointCloud<PointXYZRGB>::Ptr> pointClouds;
	vector<vector<geometry_msgs::Pose> > graspLists;

	PointCloud<PointXYZRGB>::Ptr baseCloudPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr targetCloudPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);
	
	vector<geometry_msgs::Pose> baseGraspList;
	vector<geometry_msgs::Pose> targetGraspList;
	vector<geometry_msgs::Pose> resultGraspList;
	
	ROS_INFO("Reading point clouds...");
	bool reading = true;
	stringstream ss;
	int count = 1;
	while (reading)
	{
		ss.str("");
		ss << "grasp_" << count << ".txt";
		PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
		vector<geometry_msgs::Pose> tempGraspList;
		reading = getCloud(ss.str(), tempCloudPtr, &tempGraspList);
		if (reading)
		{
			pointClouds.push_back(tempCloudPtr);
			graspLists.push_back(tempGraspList);
			count ++;
		}
		else
			count --;
	}
	ROS_INFO("Read %d Point Clouds", count);
	
	//Filter point clouds to remove noise, translate them to the origin for easier visualization
	for (unsigned int i = 0; i < pointClouds.size(); i ++)
	{
		filterCloudOutliers(pointClouds[i], RADIUS, NUM_NEIGHBORS);
		translateToOrigin(pointClouds[i], &graspLists[i], false);
	}
	
	for (unsigned int i = 0; i < pointClouds.size() - 1; i ++)
	{
		baseCloudPtr = pointClouds[i];
		for (unsigned int j = i + 1; j < pointClouds.size(); j ++)
		{
			ROS_INFO("Merging pair %d-%d", i+1, j+1);
			
			targetCloudPtr = pointClouds[j];
			
			baseGraspList = graspLists[i];
			targetGraspList = graspLists[j];
			resultGraspList.clear();
			
			resultPtr = icpRegistration(baseCloudPtr, targetCloudPtr, baseGraspList, targetGraspList, &resultGraspList, false, true);		
		}
	}
}

void pcRegistration::registerPointcloudsGraph()
{
	vector<PointCloud<PointXYZRGB>::Ptr> pointClouds;
	vector<vector<geometry_msgs::Pose> > graspLists;

	PointCloud<PointXYZRGB>::Ptr baseCloudPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr targetCloudPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);
	
	vector<geometry_msgs::Pose> baseGraspList;
	vector<geometry_msgs::Pose> targetGraspList;
	vector<geometry_msgs::Pose> resultGraspList;
	
	ROS_INFO("Reading point clouds...");
	bool reading = true;
	stringstream ss;
	int count = 1;
	while (reading)
	{
		
		//UNCOMMENT FOR USER STUDY DATA FORMAT
		ss.str("");
		ss << "grasp_" << count << ".txt";
		PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
		vector<geometry_msgs::Pose> tempGraspList;
		reading = getCloud(ss.str(), tempCloudPtr, &tempGraspList);
		if (reading)
		{
			pointClouds.push_back(tempCloudPtr);
			graspLists.push_back(tempGraspList);
			count ++;
		}
		else
			count --;
		
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
		*/
	}
	ROS_INFO("Read %d Point Clouds", count);
	
	//Filter point clouds to remove noise, translate them to the origin for easier visualization
	for (unsigned int i = 0; i < pointClouds.size(); i ++)
	{
		filterCloudOutliers(pointClouds[i], RADIUS, NUM_NEIGHBORS);
	}
	
	bool finished = false;
	int c = 0;
	
	while (!finished)
	{
		for (unsigned int i = 0; i < pointClouds.size(); i ++)
		{
			translateToOrigin(pointClouds[i], &graspLists[i], false);
		}
		c ++;
		finished = true;
		if (pointClouds.size() > 1)
		{
			for (int i = (int)(pointClouds.size()) - 1; i >= 1; i --)
			{
				if (i < pointClouds.size())
				{
					for (int j = i - 1; j >= 0; j --)
					{
						ROS_INFO("Checking pair %d-%d", i+1, j+1);
					
						baseCloudPtr = pointClouds[i];
						targetCloudPtr = pointClouds[j];
			
						if (checkRegistration(baseCloudPtr, targetCloudPtr, false))
						{
							ROS_INFO("Registration successful!");
							
							baseGraspList = graspLists[i];
							targetGraspList = graspLists[j];
							resultGraspList.clear();
							resultPtr = icpRegistration(baseCloudPtr, targetCloudPtr, baseGraspList, targetGraspList, &resultGraspList, false, false);
							filterRedundentPoints(resultPtr, DST_THRESHOLD);
							
							pointClouds.erase(pointClouds.begin() + i);
							pointClouds.erase(pointClouds.begin() + j);
							pointClouds.push_back(resultPtr);
							
							graspLists.erase(graspLists.begin() + i);
							graspLists.erase(graspLists.begin() + j);
							graspLists.push_back(resultGraspList);
							
							i --;
							
							finished = false;
						}
					}
				}
			}
		}
	}
	
	ROS_INFO("Point cloud merging resulted in %d models, in %d passes", pointClouds.size(), c);
	
	for (unsigned int i = 0; i < pointClouds.size(); i ++)
	{
		//save model
		
		/*
		//UNCOMMENT TO SAVE AS .pcd FILE
		stringstream ss;
		ss.str("");
		ss << "model_" << i + 1 << ".pcd";
		io::savePCDFileASCII(ss.str(), *pointClouds[i]);
		*/
		
		//UNCOMMENT TO SAVE WITH USER STUDY DATA FORMAT
		sensor_msgs::PointCloud saveCloud;
		sensor_msgs::PointCloud2 tempSaveCloud;
		toROSMsg(*pointClouds[i], tempSaveCloud);
		sensor_msgs::convertPointCloud2ToPointCloud(tempSaveCloud, saveCloud);
		
		stringstream ss;
		ss << "model_" << i + 1 << ".txt";

		ofstream myfile;
		myfile.open (ss.str().c_str());
	
		myfile << "reference_frame_id: base_link" << "\npointCloud:\n\theader:\n\t\tframe_id: base_link" << "\n\tpoints: [";
		for (unsigned int j = 0; j < saveCloud.points.size(); j ++)
		{
			myfile << "[" << saveCloud.points[j].x << "," << saveCloud.points[j].y << "," << saveCloud.points[j].z << "]";
			if (j < saveCloud.points.size() - 1)
				myfile << ",";
		}
		myfile << "]\n\tchannels:";
		for (unsigned int j = 0; j < saveCloud.channels.size(); j ++)
		{
			myfile << "\n\t\tname: " << saveCloud.channels[j].name << "\n\t\tvalues: [";
			for (unsigned int k = 0; k < saveCloud.channels[j].values.size(); k ++)
			{
				myfile << saveCloud.channels[j].values[k];
				if (k < saveCloud.channels[j].values.size() - 1)
					myfile << ",";
			}
			myfile << "]";
		}
		ROS_INFO("Grasp Lists Size: %d", graspLists.size());
		ROS_INFO("Current Grasp List Size: %d", graspLists[i].size());
		for (unsigned int j = 0; j < graspLists[i].size(); j ++) 
		{
			ROS_INFO("Writing grasp %d", j);
			myfile << "\ngripperPose: " << "\n\tposition: [" << graspLists[i][j].position.x << "," << graspLists[i][j].position.y << "," << graspLists[i][j].position.z << "]\n\torientation: [" << graspLists[i][j].orientation.x << "," << graspLists[i][j].orientation.y << "," << graspLists[i][j].orientation.z << "," << graspLists[i][j].orientation.w << "]";
		}
	
		myfile.close();
		
		ROS_INFO("Finished writing file model_%d", i);
		
		//visualize model
		/*
		visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(0, 0, 0);
		visualization::PointCloudColorHandlerRGBField<PointXYZRGB> colorHandler1(pointClouds[i]);
		viewer.addPointCloud(pointClouds[i], colorHandler1, "base point cloud");
		
		
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			pcl_sleep(0.01);
		}
		*/
	}
}

PointCloud<PointXYZRGB>::Ptr pcRegistration::icpRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, vector<geometry_msgs::Pose> baseGrasps, vector<geometry_msgs::Pose> targetGrasps, vector<geometry_msgs::Pose> *resultGrasps, bool scoreFiltered, bool debug)
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

bool pcRegistration::checkRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, bool debug)
{
	IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
	icp.setInputCloud(targetCloudPtr);
	icp.setInputTarget(baseCloudPtr);
	PointCloud<PointXYZRGB>::Ptr targetTransformedPtr(new PointCloud<PointXYZRGB>);
	icp.align(*targetTransformedPtr);
	float icpScore = icp.getFitnessScore();
	//ROS_INFO_STREAM("ICP convergence score: " << icpScore);
	
	float dstError = calculateRegistrationMetricDstError(baseCloudPtr, targetTransformedPtr);
	float overlap = calculateRegistrationMetricOverlap(baseCloudPtr, targetTransformedPtr, .005);
	float avgColorDiff = calculateRegistrationMetricColorRange(baseCloudPtr, targetTransformedPtr);
	float maxDstDiff = calculateRegistrationMetricDistance(baseCloudPtr, targetTransformedPtr);
	//ROS_INFO("Calculated distance error score: %f", dstError);
	//ROS_INFO("Calculated overlap score: %f", overlap);
	
	bool result = classifyMerge(overlap, maxDstDiff, dstError, avgColorDiff);
	
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

float pcRegistration::calculateRegistrationMetricDstError(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
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

float pcRegistration::calculateRegistrationMetricOverlap(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, float dstThreshold)
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

	return score;
}

float pcRegistration::calculateRegistrationMetricColorRange(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
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

float pcRegistration::calculateRegistrationMetricDistance(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
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

void pcRegistration::filterCloudOutliers(PointCloud<PointXYZRGB>::Ptr cloudPtr, double radius, int numNeighborThreshold, bool debug)
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

void pcRegistration::filterRedundentPoints(PointCloud<PointXYZRGB>::Ptr cloudPtr, double dstThreshold)
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

void pcRegistration::translateToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, vector<geometry_msgs::Pose> *grasps, bool debug)
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
	
	//transform point cloud
	Eigen::Matrix4f transform;
	transform << 1, 0, 0, -x,
				 0, 1, 0, -y,
				 0, 0, 1, -z,
				 0, 0, 0, 1;
	transformPointCloud(*cloudPtr, *cloudPtr, transform);
	
	//transform grasps
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

void pcRegistration::publishTest()
{
	baseCloudPublisher.publish(baseCloud);
	targetCloudPublisher.publish(targetCloud);
}

bool pcRegistration::classifyMerge(float overlap, float maxDstDiff, float dstError, float avgColorDiff)
{
	if (overlap <= .795576)
	{
		if (maxDstDiff <= .002273)
		{
			if (dstError <= .053681)
				return false;
			else
				return true;
		}
		else
			return false;
	}
	else
	{
		if (avgColorDiff <= 91.010641)
		{
			if (maxDstDiff <= .000304)
				return false;
			else
				return true;
		}
		else
			return false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pc_registration");
	
	MYSQL mysql;
	
	mysql_init(&mysql);
	
	pcRegistration pcr;
	
	//pcr.pairwiseRegisterPointclouds();
	pcr.registerPointcloudsGraph();
	
	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		ros::spinOnce();
		pcr.publishTest();
		loop_rate.sleep();
	}
	
	return 0;
}
