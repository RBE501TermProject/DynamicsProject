#include <ros/ros.h>
#include <grasp_learning/ReadGrasp.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <boost/thread/thread.hpp>
#include <stdlib.h>

#include <mysql/mysql.h>

//#include <pcl/common/impl/transforms.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/ros/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

//Point Cloud Filtering Constants
#define RADIUS .01
#define NUM_NEIGHBORS 23
#define DST_THRESHOLD .00075

//NARF Constants
#define ANGULAR_RESOLUTION 0.001745
#define SUPPORT_SIZE .01
#define NOISE_LEVEL 0.0
#define MIN_RANGE 0.0
#define BORDER_SIZE 1

//SIFT Constants
#define MIN_SCALE .001
#define OCTAVES 3
#define SCALES_PER_OCTAVE 3
#define MIN_CONTRAST 1.0

class pcRegistration
{
public:
	//ROS publishers, subscribers, and action servers
	ros::NodeHandle n;
	
	ros::Publisher baseCloudPublisher;
	ros::Publisher targetCloudPublisher;
	ros::ServiceClient readGraspClient;
	
	//Point clouds
	sensor_msgs::PointCloud2 baseCloud;
	sensor_msgs::PointCloud2 targetCloud;
	
	//tf
	tf::TransformListener tfListener;
	tf::TransformBroadcaster tfBroadcaster;
	
	/**
	 * Constructor
	 */
	pcRegistration();
	
	/**
	 * Reads a point cloud from the given filename and converts it into pcl::PointXYZRGB format
	 * @param filename the name of the file to read the pointcloud from
	 * @param pointcloudOut pointer to the pcl point cloud where file's point cloud will be read to
	 * @param graspListOut pointer to the grasp list where the file's grasps will be read to
	 * @return true on success, false if the file doesn't exist or is formatted incorrectly
	 */
	bool getCloud(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloudOut, std::vector<geometry_msgs::Pose> *graspListOut);
	
	/**
	 * Register all point cloud pairs
	 */
	void pairwiseRegisterPointclouds();
	
	void registerPointcloudsGraph();
	
	/**
	 * Run pcl's icp on a base and target point cloud
	 * @param baseCloudPtr pointer to the point cloud that the target should be transformed to
	 * @param targetCloudPtr pointer to the point cloud that will be merged with the base cloud
	 * @param baseGrasps list of grasps associated with the base point cloud
	 * @param targetGrasps list of grasps associated with the target point cloud
	 * @param resultGrasps pointer to the list of grasps that will be filled with transformed grasps
	 * @param debug show the merged point clouds in a visualizer
	 * @return a pointer to the merged point cloud
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr, std::vector<geometry_msgs::Pose> baseGrasps, std::vector<geometry_msgs::Pose> targetGrasps, std::vector<geometry_msgs::Pose> *resultGrasps, bool scoreFiltered = false, bool debug = false);
	
	bool checkRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr, bool debug);
	
	/**
	 * Calculate a metric for how successful the registration was based on distance error
	 * @param baseCloudPtr pointer to the point cloud that the target was transformed to
	 * @param targetCloudPtr pointer to the point cloud that was transformed to the base cloud
	 * @return a score representing the success of the registration
	 */
	float calculateRegistrationMetricDstError(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr);
	
	/**
	 * Calculate a metric for how successful the registration was based on overlap
	 * @param baseCloudPtr pointer to the point cloud that the target was transformed to
	 * @param targetCloudPtr pointer to the point cloud that was transformed to the base cloud
	 * @return a score representing the success of the registration
	 */
	float calculateRegistrationMetricOverlap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr, float dstThreshold);
	
	float calculateRegistrationMetricColorRange(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr);
	
	float calculateRegistrationMetricDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr);
	
	/**
	 * Filters point cloud outliers if they have less neighbors than the neighbor threshold within a given radius
	 * @param cloudPtr pointer to the point cloud to be filtered
	 * @param radius the radius to search for neighbors within
	 * @param numNeighborThreshold minimum number of neighbors required to keep the point
	 * @param debug display filtered point cloud in visualizer if true
	 */
	
	void filterCloudOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, double radius, int numNeighborThreshold, bool debug = false);
	
	/**
	 * Removes extra points that are within a given threshold of other points to keep the point cloud size manageable
	 * @param cloudPtr pointer to the point cloud to be filtered
	 * @param dstThreshold the minimum distance between neighboring points at which to keep a point
	 */
	void filterRedundentPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, double dstThreshold);
	
	/**
	 * Translates a point cloud so that it's average point lies on the origin
	 * @param cloudPtr pointer to the point cloud to be translated
	 * @param grasps pointer to the grasps corresponding to the point cloud to be translated
	 * @param debug display translated point cloud in visualizer if true
	 */
	void translateToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::vector<geometry_msgs::Pose> *grasps, bool debug = false);
	
	/**
	 * Point cloud publishing for ROS visualization
	 */
	void publishTest();
	
	bool classifyMerge(float overlap, float maxDstDiff, float dstError, float avgColorDiff);
};
