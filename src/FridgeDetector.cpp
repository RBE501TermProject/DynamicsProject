#include <DynamicsProject/FridgeDetector.h>

using namespace std;
using namespace cv;

FridgeDetector::FridgeDetector()
{
	graspFridgeServer = n.advertiseService("fridge_detector/grasp_fridge", &FridgeDetector::graspFridge, this);
	releaseFridgeServer = n.advertiseService("fridge_detector/release_fridge", &FridgeDetector::releaseFridge, this);
	
	imageSubscriber = n.subscribe("/head_mount_kinect/rgb/image_color", 1, &FridgeDetector::imageCallback, this);
}

bool FridgeDetector::graspFridge(DynamicsProject::GraspFridge::Request &req, DynamicsProject::GraspFridge::Response &res)
{
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fridge_detector");
	
	FridgeDetector fd;
	
	ros::Rate loopRate(30);
	while (ros::ok())
	{
		ros::spinOnce();
		loopRate.sleep();
	}
	
	return 0;
}
