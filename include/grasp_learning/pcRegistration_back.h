#include <ros/ros.h>
#include <grasp_learning/ReadGrasp.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/thread/thread.hpp>
#include <stdlib.h>

//#include <pcl/common/impl/transforms.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/range_image_border_extractor.h>
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
#define DST_THRESHOLD .0005

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
	
	/**
	 * Constructor
	 */
	pcRegistration();
	
	/**
	 * Reads a point cloud from the given filename and converts it into pcl::PointXYZRGB format
	 * @param filename the name of the file to read the pointcloud from
	 * @param pointcloudOut pointer to the pcl point cloud where file will be read to
	 * @return true on success, false if the file doesn't exist or is formatted incorrectly
	 */
	bool getCloud(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloudOut);
	
	/**
	 * Calculate keypoints using Normal Aligned Radial Features
	 * @param pointCloudPtr pointer to the point cloud to extract keypoints from
	 * @param keypointsOut pointer to a point cloud to contain the extracted keypoints
	 * @param debug display point clouds in visualizer if true
	 */
	void calcKeyPointsNARF(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypointsOut, bool debug = false);
	
	/**
	 * Calculate keypoints using Scale Invariant Feature Transform
	 * @param pointCloudPtr pionter to the point cloud to extract keypoints from
	 * @param keypointsOut pointer to a point cloud to contain the extracted keypoints
	 * @param debug display point clouds in visualizer if true
	 */
	void calcKeyPointsSIFT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypointsOut, bool debug = false);
	
	/**
	 * Compute FPFH feature data based on keypoints calculated previously
	 * (called after either calcKeyPointsNARF() or calcKeyPointsSift())
	 * @param pointCloudPtr pointer to the entire point cloud which features were extracted from
	 * @param keypointsPtr pointer to the keypoints to compute features for
	 * @param featuresOut pointer to store computed features for each keypoint
	 */
	void calcFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypointsPtr, pcl::PointCloud<pcl::FPFHSignature33>::Ptr featuresOut);
	
	/**
	 * Compute PFH feature data based on keypoints calculated previously
	 * (called after either calcKeyPointsNARF() or calcKeyPointsSift())
	 * @param pointCloudPtr pointer to the entire point cloud which features were extracted from
	 * @param keypointsPtr pointer to the keypoints to compute features for
	 * @param pointer to store computed features for each keypoint
	 */
	void calcFeaturesPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypointsPtr, pcl::PointCloud<pcl::PFHSignature125>::Ptr featuresOut);
	
	/**
	 * Find correspondences between the base and target point cloud features
	 * @param baseFeaturesPtr pointer to the base point cloud's features
	 * @param targetFeaturesPtr pointer to the target point cloud's features
	 * @param baseKeypointsPtr pointer to the base point cloud's keypoints
	 * @param targetKeypointsPtr pointer to the target point cloud's keypoints
 	 * @param pointer to store correspondences in after finding initial correspondences and rejecting bad correspondences
	 * @param debug display point clouds in visualizer if true
	 */
	void findCorrespondence(pcl::PointCloud<pcl::FPFHSignature33>::Ptr baseFeaturesPtr, pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeaturesPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseKeypointsPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeypointsPtr, pcl::Correspondences *correspondencesPtrOut, bool debug = false);
	
	/**
	 * Find correspondences between the base and target point cloud features, using PFH features instead of FPFH
	 * @param baseFeaturesPtr pointer to the base point cloud's features
	 * @param targetFeaturesPtr pointer to the target point cloud's features
	 * @param baseKeypointsPtr pointer to the base point cloud's keypoints
	 * @param targetKeypointsPtr pointer to the target point cloud's keypoints
	 * @param pointer to store correspondences in after finding initial correspondences and rejecting bad correspondences
	 * @param debug display point clouds in visualizer if true
	 */
	void findCorrespondence2(pcl::PointCloud<pcl::PFHSignature125>::Ptr baseFeaturesPtr, pcl::PointCloud<pcl::PFHSignature125>::Ptr targetFeaturesPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseKeypointsPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeypointsPtr, pcl::Correspondences *correspondencesPtrOut, bool debug = false);
	
	/**
	 * Determine a transformation matrix to minimize error between point correspondences
	 * @param baseKeypointsPtr pointer to the keypoints of the base point cloud
	 * @param targetKeypiontsPtr pointer to the keypoints of the target point cloud
	 * @param correspondencesPtr pointer to the correspondences between points of the base and target 
	 * 		point clouds' keypoints
	 * @param pointer to a transformation matrix to store the transform from the base point cloud to the target point cloud
	 */
	void computeTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseKeypointsPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetKeypointsPtr, pcl::Correspondences *correspondencesPtr, Eigen::Matrix4f *transformPtrOut);
	
	/**
	 * Transform the target point cloud and combine it with the base point cloud
	 * @param baseCloudPtr pointer to the base point cloud
	 * @param targetCloudPtr pointer to the target point cloud to be transformed
	 * @param transformPtr pointer to a transformation matrix used to transform the target point cloud
	 * @param debug display transformed and combined point cloud in the visualizer if true
	 * @return a new point cloud made from combining the base point cloud and the transformed target point cloud
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,  pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr, Eigen::Matrix4f *transformPtr, bool debug = false);
	
	/**
	 * Register all point cloud pairs
	 */
	void pairwiseRegisterPointclouds();
	
	/**
	 * Iteratively register all point clouds
	 */
	void simpleIterativeRegisterPointclouds();
	
	/**
	 * Run pcl's icp on a base and target point cloud
	 * @param baseCloudPtr pointer to the point cloud that the target should be tranformed to
	 * @param targetCloudPtr pointer to the point cloud that will be merged with the base cloud
	 * @param debug show the merged point clouds in a visualizer
	 * @return a pointer to the merged point cloud
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr, bool scoreFiltered = false, bool debug = false);
	
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
	 * @param debug display translated point cloud in visualizer if true
	 */
	void translateToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, bool debug = false);
	
	/**
	 * TEST CODE!!!
	 */
	void RGBtoHSV(float r, float g, float b, float *h, float *s, float *v);
	
	/**
	 * Point cloud publishing for ROS visualization
	 */
	void publishTest();
	
	bool classifyMerge(float overlap, float maxDstDiff, float dstError, float avgColorDiff);
};
