#include "grasp_learning/pcRegistration.h"

using namespace std;
using namespace pcl;

pcRegistration::pcRegistration()
{
	baseCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("pc_registration/base_cloud", 1);
	targetCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("pc_registration/target_cloud", 1);
	readGraspClient = n.serviceClient<grasp_learning::ReadGrasp>("grasp_reader/read_grasps");
}

bool pcRegistration::getCloud(std::string filename, PointCloud<PointXYZRGB>::Ptr pointcloudOut)
{	
	grasp_learning::ReadGrasp srv;
	srv.request.grasp_entry = filename;

	readGraspClient.call(srv);

	if (srv.response.success)
	{	
		sensor_msgs::convertPointCloudToPointCloud2(srv.response.grasp.pointCloud, baseCloud);
		fromROSMsg(baseCloud, *pointcloudOut);
		baseCloudPublisher.publish(baseCloud);
	}

	return srv.response.success;	
}
	
void pcRegistration::calcKeyPointsNARF(PointCloud<PointXYZRGB>::Ptr pointCloudPtr, PointCloud<PointXYZRGB>::Ptr keypointsOut, bool debug)
{	
	// Create RangeImage from the PointCloud
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	RangeImage::CoordinateFrame coordinateFrame = RangeImage::CAMERA_FRAME;
	
	boost::shared_ptr<RangeImage> rangeImagePtr(new RangeImage);
	RangeImage& rangeImage = *rangeImagePtr;
	rangeImage.createFromPointCloud(*pointCloudPtr, ANGULAR_RESOLUTION, deg2rad(360.0f), deg2rad(180.0f),
		sensorPose, coordinateFrame, NOISE_LEVEL, MIN_RANGE, BORDER_SIZE);
	
	// Extract NARF keypoints
	RangeImageBorderExtractor rangeImageBorderExtractor;
	NarfKeypoint narf;
	narf.setRangeImageBorderExtractor(&rangeImageBorderExtractor);
	narf.setRangeImage(&rangeImage);
	narf.getParameters().support_size = SUPPORT_SIZE;

	PointCloud<int> keypointIndices;
	narf.compute(keypointIndices);
	
	ROS_INFO("Found %d NARF Keypoints.", keypointIndices.points.size());

	// Convert keypoints to PointXYZRGB
	keypointsOut->points.resize(keypointIndices.points.size());
	for (size_t i = 0; i < keypointIndices.points.size(); i++)
	{
		keypointsOut->points[i].getVector3fMap() = rangeImage.points[keypointIndices.points[i]].getVector3fMap();
	}
	
	// Show keypoints in 3D viewer
	if (debug)
	{
		visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(255, 255, 255);
		visualization::PointCloudColorHandlerCustom<PointWithRange> rangeColorHandler(rangeImagePtr, 0, 0, 0);
		viewer.addPointCloud(rangeImagePtr, rangeColorHandler, "range image");
		viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
		viewer.initCameraParameters();
		visualization::PointCloudColorHandlerCustom<PointXYZRGB> keypointsColorHandler(keypointsOut, 0, 255, 0);
		viewer.addPointCloud<PointXYZRGB>(keypointsOut, keypointsColorHandler, "keypoints");
		viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
	
		// Main loop
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			pcl_sleep(0.01);
		}
	}
}

void pcRegistration::calcKeyPointsSIFT(PointCloud<PointXYZRGB>::Ptr pointCloudPtr, PointCloud<PointXYZRGB>::Ptr keypointsOut, bool debug)
{	
	// Extract SIFT keypoints
	SIFTKeypoint<PointXYZRGB, PointWithScale> sift;
	PointCloud<PointWithScale>::Ptr siftFeaturesPtr(new PointCloud<PointWithScale>);
	sift.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr(new search::KdTree<PointXYZRGB>));
	sift.setScales(MIN_SCALE, OCTAVES, SCALES_PER_OCTAVE);
	sift.setMinimumContrast(MIN_CONTRAST);
	sift.setInputCloud(pointCloudPtr);
	sift.compute(*siftFeaturesPtr);
	
	copyPointCloud(*siftFeaturesPtr, *keypointsOut);
	
	ROS_INFO("Found %d SIFT Keypoints.", siftFeaturesPtr->points.size());
	
	// Add SIFT keypoints to 3D viewer
	if (debug)
	{
		visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(0, 0, 0);
		visualization::PointCloudColorHandlerRGBField<PointXYZRGB> cloudColorHandler(pointCloudPtr);
		viewer.addPointCloud(pointCloudPtr, cloudColorHandler, "original point cloud");
		viewer.initCameraParameters();
		visualization::PointCloudColorHandlerCustom<PointWithScale> siftColorHandler(siftFeaturesPtr, 0, 255, 0);
		viewer.addPointCloud(siftFeaturesPtr, siftColorHandler, "SIFT features");	
	
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			pcl_sleep(0.01);
		}
	}
}

void pcRegistration::calcFeatures(PointCloud<PointXYZRGB>::Ptr pointCloudPtr, PointCloud<PointXYZRGB>::Ptr keypointsPtr, PointCloud<FPFHSignature33>::Ptr featuresOut)
{
	// Calculate surface normals
	NormalEstimation<PointXYZRGB, Normal> normalEstimator;
	normalEstimator.setInputCloud(pointCloudPtr);
	normalEstimator.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr(new search::KdTree<PointXYZRGB>));
	PointCloud<Normal>::Ptr normalsPtr(new PointCloud<Normal>);
	normalEstimator.setRadiusSearch(.01);
	normalEstimator.compute(*normalsPtr);
	
	// Compute FPFH features
	FPFHEstimation<PointXYZRGB, Normal, FPFHSignature33> fpfh;
	fpfh.setSearchSurface(pointCloudPtr);
	fpfh.setInputCloud(keypointsPtr);
	fpfh.setInputNormals(normalsPtr);
	fpfh.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr(new search::KdTree<PointXYZRGB>));
	fpfh.setRadiusSearch(0.02);
	fpfh.compute(*featuresOut);
}

void pcRegistration::calcFeaturesPFH(PointCloud<PointXYZRGB>::Ptr pointCloudPtr, PointCloud<PointXYZRGB>::Ptr keypointsPtr, PointCloud<PFHSignature125>::Ptr featuresOut)
{
	// Calculate surface normals
	NormalEstimation<PointXYZRGB, Normal> normalEstimator;
	normalEstimator.setInputCloud(pointCloudPtr);
	normalEstimator.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr(new search::KdTree<PointXYZRGB>));
	PointCloud<Normal>::Ptr normalsPtr(new PointCloud<Normal>);
	normalEstimator.setRadiusSearch(.03);
	normalEstimator.compute(*normalsPtr);
	
	// Compute PFH features
	PFHEstimation<PointXYZRGB, Normal, PFHSignature125> pfh;
	pfh.setRadiusSearch(0.08);
	pfh.setSearchSurface(pointCloudPtr);
	pfh.setInputCloud(keypointsPtr);
	pfh.setInputNormals(normalsPtr);
	pfh.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr(new search::KdTree<PointXYZRGB>));
	pfh.setRadiusSearch(0.05);
	pfh.compute(*featuresOut);
}

void pcRegistration::findCorrespondence(PointCloud<FPFHSignature33>::Ptr baseFeaturesPtr, PointCloud<FPFHSignature33>::Ptr targetFeaturesPtr, PointCloud<PointXYZRGB>::Ptr baseKeypointsPtr, PointCloud<PointXYZRGB>::Ptr targetKeypointsPtr, Correspondences *correspondencesPtrOut, bool debug)
{
	//Get initial correspondences
	Correspondences initialCorrespondences;
	registration::CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> correspondenceEstimator;
	correspondenceEstimator.setInputCloud(baseFeaturesPtr);
	correspondenceEstimator.setInputTarget(targetFeaturesPtr);
	//correspondenceEstimator.determineCorrespondences(initialCorrespondences);
	correspondenceEstimator.determineReciprocalCorrespondences(initialCorrespondences);
	
	//Correspondence rejection
	registration::CorrespondenceRejectorSampleConsensus<PointXYZRGB> correspondenceRejector;
	correspondenceRejector.setInputCloud(baseKeypointsPtr);
	correspondenceRejector.setTargetCloud(targetKeypointsPtr);
	correspondenceRejector.getRemainingCorrespondences(initialCorrespondences, *correspondencesPtrOut);

	// Open 3D viewer and add point clouds and correspondences
	if (debug)
	{
		visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(0, 0, 0);
		visualization::PointCloudColorHandlerCustom<PointXYZRGB> colorHandler1(baseKeypointsPtr, 0, 255, 0);
		visualization::PointCloudColorHandlerCustom<PointXYZRGB> colorHandler2(baseKeypointsPtr, 0, 255, 0);
		viewer.addPointCloud(baseKeypointsPtr, colorHandler1, "base point cloud");
		viewer.addPointCloud(targetKeypointsPtr, colorHandler2, "target point cloud");
		viewer.addCorrespondences<PointXYZRGB>(baseKeypointsPtr, targetKeypointsPtr, *correspondencesPtrOut, "correspondences");
		viewer.initCameraParameters();
		
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			pcl_sleep(0.01);
		}
	}
}

void pcRegistration::findCorrespondence2(PointCloud<PFHSignature125>::Ptr baseFeaturesPtr, PointCloud<PFHSignature125>::Ptr targetFeaturesPtr, PointCloud<PointXYZRGB>::Ptr baseKeypointsPtr, PointCloud<PointXYZRGB>::Ptr targetKeypointsPtr, Correspondences *correspondencesPtrOut, bool debug)
{
	//Get initial correspondences
	Correspondences initialCorrespondences;
	registration::CorrespondenceEstimation<PFHSignature125, PFHSignature125> correspondenceEstimator;
	correspondenceEstimator.setInputCloud(baseFeaturesPtr);
	correspondenceEstimator.setInputTarget(targetFeaturesPtr);
	//correspondenceEstimator.determineCorrespondences(initialCorrespondences);
	correspondenceEstimator.determineReciprocalCorrespondences(initialCorrespondences);
	
	//Correspondence rejection
	registration::CorrespondenceRejectorSampleConsensus<PointXYZRGB> correspondenceRejector;
	correspondenceRejector.setInputCloud(baseKeypointsPtr);
	correspondenceRejector.setTargetCloud(targetKeypointsPtr);
	correspondenceRejector.getRemainingCorrespondences(initialCorrespondences, *correspondencesPtrOut);

	// Open 3D viewer and add point clouds and correspondences
	if (debug)
	{
		visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(0, 0, 0);
		visualization::PointCloudColorHandlerCustom<PointXYZRGB> colorHandler1(baseKeypointsPtr, 0, 255, 0);
		visualization::PointCloudColorHandlerCustom<PointXYZRGB> colorHandler2(targetKeypointsPtr, 0, 255, 0);
		viewer.addPointCloud(baseKeypointsPtr, colorHandler1, "base point cloud");
		viewer.addPointCloud(targetKeypointsPtr, colorHandler2, "target point cloud");
		viewer.addCorrespondences<PointXYZRGB>(baseKeypointsPtr, targetKeypointsPtr, *correspondencesPtrOut, "correspondences");
		viewer.initCameraParameters();
		
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			pcl_sleep(0.01);
		}
	}
}

void pcRegistration::computeTransformation(PointCloud<PointXYZRGB>::Ptr baseKeypointsPtr, PointCloud<PointXYZRGB>::Ptr targetKeypointsPtr, Correspondences *correspondencesPtr, Eigen::Matrix4f *transformPtrOut)
{
	registration::TransformationEstimationSVD<PointXYZRGB, PointXYZRGB> transformEstimator;
	transformEstimator.estimateRigidTransformation(*targetKeypointsPtr, *baseKeypointsPtr, *correspondencesPtr, *transformPtrOut);
}

PointCloud<PointXYZRGB>::Ptr pcRegistration::transformPointcloud(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, Eigen::Matrix4f *transformPtr, bool debug)
{
	PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr targetTransformedPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB> result = *resultPtr;
	
	transformPointCloud(*targetCloudPtr, *targetTransformedPtr, *transformPtr);
	result = *baseCloudPtr + *targetTransformedPtr;
	
	if (debug)
	{
		visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(0, 0, 0);
		//visualization::PointCloudColorHandlerCustom<PointXYZRGB> colorHandler1(baseCloudPtr, 0, 255, 0);
		//visualization::PointCloudColorHandlerCustom<PointXYZRGB> colorHandler2(targetTransformedPtr, 0, 255, 0);
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
	
	//TEMPORARY!!!
	resultPtr = targetTransformedPtr;
	
	return resultPtr;
}

void pcRegistration::pairwiseRegisterPointclouds()
{
	vector<PointCloud<PointXYZRGB>::Ptr> pointClouds;

	PointCloud<PointXYZRGB>::Ptr baseCloudPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr targetCloudPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);
	
	ROS_INFO("Reading point clouds...");
	bool reading = true;
	stringstream ss;
	int count = 1;
	while (reading)
	{
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
	}
	ROS_INFO("Read %d Point Clouds", count);
	
	//Filter point clouds to remove noise, translate them to the origin for easier visualization
	for (unsigned int i = 0; i < pointClouds.size(); i ++)
	{
		filterCloudOutliers(pointClouds[i], RADIUS, NUM_NEIGHBORS);
		translateToOrigin(pointClouds[i], false);
	}
	
	for (unsigned int i = 0; i < pointClouds.size() - 1; i ++)
	{
		baseCloudPtr = pointClouds[i];
		for (unsigned int j = i + 1; j < pointClouds.size(); j ++)
		{
			ROS_INFO("Merging pair %d-%d", i+1, j+1);
			
			targetCloudPtr = pointClouds[j];
			
			resultPtr = icpRegistration(baseCloudPtr, targetCloudPtr, false, true);		
		}
	}
}

void pcRegistration::simpleIterativeRegisterPointclouds()
{
	vector<PointCloud<PointXYZRGB>::Ptr> pointClouds;

	PointCloud<PointXYZRGB>::Ptr baseCloudPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr targetCloudPtr(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);
	
	ROS_INFO("Reading point clouds...");
	bool reading = true;
	stringstream ss;
	int count = 1;
	while (reading)
	{
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
	}
	ROS_INFO("Read %d Point Clouds", count);
	
	//Filter point clouds to remove noise, translate them to the origin for easier visualization
	for (unsigned int i = 0; i < pointClouds.size(); i ++)
	{
		filterCloudOutliers(pointClouds[i], RADIUS, NUM_NEIGHBORS);
		translateToOrigin(pointClouds[i], false);
	}
	
	ROS_INFO("Initializing with point cloud 1...");
	baseCloudPtr = pointClouds[0];
	
	for (unsigned int i = 1; i < pointClouds.size(); i ++)
	{
		ROS_INFO("Attempting to add point cloud %d...", i+1);
			
		targetCloudPtr = pointClouds[i];
		
		resultPtr = icpRegistration(baseCloudPtr, targetCloudPtr, true, true);
		
		ROS_INFO("current size...%d", resultPtr->size());
		
		if (resultPtr->size() <= 0)
		{
			ROS_INFO("Registration failed for point cloud %d.", i+1);
		}
		else
		{
			baseCloudPtr = resultPtr;
		}
	}
}

PointCloud<PointXYZRGB>::Ptr pcRegistration::icpRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, bool scoreFiltered, bool debug)
{
	IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
	icp.setInputCloud(targetCloudPtr);
	icp.setInputTarget(baseCloudPtr);
	PointCloud<PointXYZRGB>::Ptr targetTransformedPtr(new PointCloud<PointXYZRGB>);
	icp.align(*targetTransformedPtr);
	float icpScore = icp.getFitnessScore();
	//ROS_INFO_STREAM("ICP convergence score: " << icpScore);
	
	float dstError = calculateRegistrationMetricDstError(baseCloudPtr, targetTransformedPtr);
	//ROS_INFO("Calculated distance error score: %f", dstError);
	float overlap = calculateRegistrationMetricOverlap(baseCloudPtr, targetTransformedPtr, .005);
	float avgColorDiff = calculateRegistrationMetricColorRange(baseCloudPtr, targetTransformedPtr);
	float maxDstDiff = calculateRegistrationMetricDistance(baseCloudPtr, targetTransformedPtr);
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
		//Eigen::Matrix4f transform = icp.getFinalTransformation();
	
		//transformPointCloud(*targetCloudPtr, *targetTransformedPtr, transform);
		result = *baseCloudPtr + *targetTransformedPtr;
	
		filterRedundentPoints(resultPtr, DST_THRESHOLD);
	}
	
	classifyMerge(overlap, maxDstDiff, dstError, avgColorDiff);
	
	if (debug)
	{
		visualization::PCLVisualizer viewer("3D Viewer");
		viewer.setBackgroundColor(0, 0, 0);
		//visualization::PointCloudColorHandlerCustom<PointXYZRGB> colorHandler1(baseCloudPtr, 0, 255, 0);
		//visualization::PointCloudColorHandlerCustom<PointXYZRGB> colorHandler2(targetTransformedPtr, 0, 255, 0);
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
				
				float h1, s1, v1, h2, s2, v2;
				RGBtoHSV(searchPoint.r/255.0f, searchPoint.g/255.0f, searchPoint.b/255.0f, &h1, &s1, &v1);
				RGBtoHSV(point.r/255.0f, point.g/255.0f, point.b/255.0f, &h2, &s2, &v2);
				//colorDistance += sqrt(pow(h1 - h2, 2) + pow(s1 - s2, 2) + pow(v1 - v2, 2));
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
	ROS_INFO("%f", colorError);
	ROS_INFO("%f", score);
	
	return score;
}

float pcRegistration::calculateRegistrationMetricColorRange(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{
	//float minr = 999, ming = 999, minb = 999;
	//float maxr = 0, maxg = 0, maxb = 0;
	float avgr = 0, avgg = 0, avgb = 0;
	//float rRange1, rRange2, gRange1, gRange2, bRange1, bRange2;
	
	for (unsigned int i = 0; i < targetCloudPtr->size(); i ++)
	{
		PointXYZRGB point = targetCloudPtr->at(i);
		
		/*
		if (point.r < minr)
			minr = point.r;
		if (point.r > maxr)
			maxr = point.r;
		if (point.g < ming)
			ming = point.g;
		if (point.g > maxg)
			maxg = point.g;
		if (point.b < minb)
			minb = point.b;
		if (point.b > maxb)
			maxb = point.b;
		*/
			
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
	
	//rRange1 = maxr - minr;
	//gRange1 = maxg - ming;
	//bRange1 = maxb - minb;
	
	/*
	minr = 999;
	ming = 999;
	minb = 999;
	maxr = 0;
	maxg = 0;
	maxb = 0;
	*/
	
	for (unsigned int i = 0; i < baseCloudPtr->size(); i ++)
	{
		PointXYZRGB point = baseCloudPtr->at(i);
		
		/*
		if (point.r < minr)
			minr = point.r;
		if (point.r > maxr)
			maxr = point.r;
		if (point.g < ming)
			ming = point.g;
		if (point.g > maxg)
			maxg = point.g;
		if (point.b < minb)
			minb = point.b;
		if (point.b > maxb)
			maxb = point.b;
		*/
			
		avgr += point.r;
		avgg += point.g;
		avgb += point.b;
	}
	
	//ROS_INFO("Base AVG Red: %f", avgr /= targetCloudPtr->size());
	//ROS_INFO("Base AVG Green: %f", avgg /= targetCloudPtr->size());
	//ROS_INFO("Base AVG Blue: %f", avgb /= targetCloudPtr->size());
	
	float avg2 = (avgr + avgg + avgb) / baseCloudPtr->size();
	
	//rRange2 = maxr - minr;
	//gRange2 = maxg - ming;
	//bRange2 = maxb - minb;
	
	//ROS_INFO("Target Red Range: %f", rRange1);
	//ROS_INFO("Base Red Range: %f", rRange2);
	//ROS_INFO("Target Green Range: %f", gRange1);
	//ROS_INFO("Base Green Range: %f", gRange2);
	//ROS_INFO("Target Blue Range: %f", bRange1);
	//ROS_INFO("Base Blue Range: %f", bRange2);
}

float pcRegistration::calculateRegistrationMetricDistance(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{
	//ROS_INFO("Target size: %d", targetCloudPtr->size());
	//ROS_INFO("Base size: %d", baseCloudPtr->size());
	
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

//*************************************************************************************************//
//*******************************Test Code*********************************************************//
//*************************************************************************************************//
void pcRegistration::RGBtoHSV(float r, float g, float b, float *h, float *s, float *v)
{
	float min, max, delta;
	
	min = r;
	if (g < min)
		min = g;
	if (b < min)
		min = b;
	max = r;
	if (g > max)
		max = g;
	if (b > max)
		max = b;
		
	*v = max;
	
	delta = max - min;
	
	if (max != 0)
		*s = delta / max;
	else
	{
		*s = 0;
		*h = -1;
		return;
	}
	
	if (r == max)
		*h = (g - b)/delta;
	else if (g == max)
		*h = 2 + (b-r) / delta;
	else
		*h = 4 + (r - g)/delta;
	*h *= 60;
	if (*h < 0)
		*h += 360;
}
//*************************************************************************************************//
//*****************************End Test Code*******************************************************//
//*************************************************************************************************//

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

void pcRegistration::translateToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, bool debug)
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
	
	pcRegistration pcr;
	
	pcr.pairwiseRegisterPointclouds();
	//pcr.simpleIterativeRegisterPointclouds();
	
	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		ros::spinOnce();
		pcr.publishTest();
		loop_rate.sleep();
	}
	
	return 0;
}
