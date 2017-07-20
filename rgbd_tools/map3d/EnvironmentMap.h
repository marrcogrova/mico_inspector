//
//
//
//
//


#ifndef ENVIRONMENTMAP_H_
#define ENVIRONMENTMAP_H_

// 666 TODO: clean includes

#include <vector>


#include <boost/make_shared.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>


/// Class used for creating an local map.
/// For more information refer to:
///		> Ramon Soria, P.; Bevec, R.; Arrue, B.C.; Ude, A.; Ollero, A.	Extracting Objects for Aerial Manipulation on UAVs Using Low Cost Stereo Sensors. Sensors 2016, 16, 700.
///
class EnvironmentMap {
public:		// Public interface
	enum eHistoryCalculation {Simple = 0, Sequential, Accurate};
	/// Internal structure to configure algorithms
	struct Params {
		// Voxeling parameters.
		float	voxelSize;

		// Filter outlier removal.
		int		outlierMeanK;
		float	outlierStdDev;
		bool	outlierSetNegative;

		// ICP-NL
		double	icpMaxTransformationEpsilon;
		double	icpEuclideanEpsilon;
		int		icpMaxIcpIterations;
		float	icpMaxCorrespondenceDistance;
		float	icpMaxAngleChangeCompared2ProvidedGuess;
		float	icpMaxTranslationChangeCompared2ProvidedGuess;

		// Pointcloud history filtering
		unsigned	historySize;

		// Euclidean clustering
		double	clusterTolerance;
		int		minClusterSize;
		int		maxClusterSize;

		// Floor extractor
		double			floorCameraMinAngle;
		double			floorCameraMaxAngle;
		double			floorDistanceThreshold;
		unsigned		floorMaxIters;
	};

	/// Basic constructor. Initialize an empty map
	EnvironmentMap();
	EnvironmentMap(Params _params);

	void params(Params _params);

	Params params() const;

	/// Remove internal pointcloud
	void clear();

	/// Add points into internal cloud.
	/// \param _cloud:
	bool addPoints(const pcl::PointCloud< pcl::PointXYZRGB>::Ptr &_cloud, const Eigen::Vector4f &_translationPrediction, const Eigen::Quaternionf &_qRotationPrediction, enum eHistoryCalculation _calculation, const double _maxFittingScore, pcl::PointCloud< pcl::PointXYZRGB>::Ptr &_addedCloud);

	/// Cluster internal point cloud and returns vector with clusters
	/// \return  
	std::vector<pcl::PointIndices> clusterCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
	std::vector<pcl::PointIndices> clusterCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &_clusters);

	/// Get point cloud
	pcl::PointCloud<pcl::PointXYZRGB> cloud();


	/// Look for planes in the given pointcloud.
	pcl::ModelCoefficients  extractFloor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);

	/// Calculate the minimal distance from the given cluster to a plane.
	double distanceToPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, const pcl::ModelCoefficients &_plane);

	/// Crop the map using a plane
	void cropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, pcl::ModelCoefficients _plane, bool _upperSide = true);

	/// Filter internal pointcloud.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);

	/// voxelate current map/pointcloud.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);

	/// point clouds can have orientation and origin data inside, but sometimes we need the Matrix4f form. 
	/// This function returns that matrix
	Eigen::Matrix4f transformationFromSensor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
	
	void updateSensorPose(const Eigen::Vector4f &_position, const Eigen::Quaternionf &_orientation);

	double fittingScore() const { return mFittingScore; }
	Eigen::Matrix4f ICPres() const { return mICPres; }
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr AlignedCloud() { return mAlignedCloud.makeShared(); }
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr GuessCloud() { return mGuessCloud.makeShared(); }
private:	// Private methods
	// Calculate transformation between two point cloud using ICP-NL algorithm.
	bool getTransformationBetweenPcs(const pcl::PointCloud<pcl::PointXYZRGB> &_newCloud,
		const pcl::PointCloud< pcl::PointXYZRGB> &_targetCloud, 
		Eigen::Matrix4f &_transformation, 
		const double _maxFittingScore,
		const Eigen::Matrix4f &_initialGuess,
		pcl::PointCloud<pcl::PointXYZRGB> &_alignedCloud);
	
	// This method use the internal voxel grid to downsample the input clouds. 
	// Then operate a 1x1 convolution between both: 
	//			exists(cloud1(x,y,z)) && exists(cloud2(x,y,z)) ? Point(x,y,z) : null
	pcl::PointCloud<pcl::PointXYZRGB> convoluteCloudsOnGrid(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud1, const pcl::PointCloud<pcl::PointXYZRGB> &_cloud2);

	bool validTransformation(const Eigen::Matrix4f & _transformation, const Eigen::Matrix4f &_guess);

	
private:	// Members
	Params	mParams;

	std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>		mCloudHistory;
	pcl::PointCloud<pcl::PointXYZRGB>						mCloud;
	pcl::VoxelGrid<pcl::PointXYZRGB>						mVoxelGrid;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>		mOutlierRemoval;
	pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB>	mPcJoiner;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> mEuclideanClusterExtraction;

	double mFittingScore = 0;
	Eigen::Matrix4f mICPres;
	pcl::PointCloud<pcl::PointXYZRGB> mAlignedCloud;
	pcl::PointCloud<pcl::PointXYZRGB> mGuessCloud;

	//history calculation options
	bool addPointsSimple(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _cloud, const Eigen::Vector4f &_translationPrediction, const Eigen::Quaternionf &_qRotationPrediction,const double _maxFittingScore, pcl::PointCloud< pcl::PointXYZRGB>::Ptr &_addedCloud);
	bool addPointsSequential(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _cloud, const double _maxFittingScore, pcl::PointCloud< pcl::PointXYZRGB>::Ptr &_addedCloud);
	bool addPointsAccurate(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _cloud, const double _maxFittingScore, pcl::PointCloud< pcl::PointXYZRGB>::Ptr &_addedCloud);
	bool transformCloudtoTargetCloudAndAddToHistory(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _cloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _target, const double _maxFittingScore, const Eigen::Matrix4f &_guess);
	pcl::PointCloud<pcl::PointXYZRGB> convoluteCloudsInQueue(std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _cloudQueue);
	Eigen::Vector3f originInverse(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
	Eigen::Quaternionf sensorInverse(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
};	// class EnvironmentMap

#endif	//	ENVIRONMENTMAP_H_
