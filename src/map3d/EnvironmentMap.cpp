//
//
//
//
//

#include <rgbd_tools/map3d/EnvironmentMap.h>

#include <opencv2/opencv.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace std;
using namespace Eigen;

//---------------------------------------------------------------------------------------------------------------------
EnvironmentMap::EnvironmentMap() {

}

//---------------------------------------------------------------------------------------------------------------------
EnvironmentMap::EnvironmentMap(EnvironmentMap::Params _params) {
	mParams = _params;
	params(mParams);
}

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::params(EnvironmentMap::Params _params) {
	mParams = _params;

	mVoxelGrid.setLeafSize(_params.voxelSize, _params.voxelSize, _params.voxelSize);

	// Init filtering class
	mOutlierRemoval.setMeanK(_params.outlierMeanK);
	mOutlierRemoval.setStddevMulThresh(_params.outlierStdDev);
	mOutlierRemoval.setNegative(_params.outlierSetNegative);

	// Init ICP-NL class
	mPcJoiner.setTransformationEpsilon (_params.icpMaxTransformationEpsilon);
	mPcJoiner.setMaxCorrespondenceDistance (_params.icpMaxCorrespondenceDistance);  
	mPcJoiner.setMaximumIterations (_params.icpMaxIcpIterations);
	mPcJoiner.setEuclideanFitnessEpsilon(_params.icpEuclideanEpsilon);

	// Init Euclidean Extraction
	mEuclideanClusterExtraction.setClusterTolerance(_params.clusterTolerance);
	mEuclideanClusterExtraction.setMinClusterSize(_params.minClusterSize);
	mEuclideanClusterExtraction.setMaxClusterSize(_params.maxClusterSize);
}

//---------------------------------------------------------------------------------------------------------------------
EnvironmentMap::Params EnvironmentMap::params() const {
	return mParams;
}

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::clear() {
	mCloud.clear();
	mCloudHistory.clear();
	mAlignedCloud.clear();
	mGuessCloud.clear();
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZRGB>::Ptr EnvironmentMap::filter(const PointCloud<PointXYZRGB>::Ptr &_cloud) {
	PointCloud<PointXYZRGB>::Ptr filteredCloud(new PointCloud<PointXYZRGB>);
	mOutlierRemoval.setInputCloud(_cloud);
	mOutlierRemoval.filter(*filteredCloud);
	return filteredCloud;
}

//---------------------------------------------------------------------------------------------------------------------
// you can only use one type of history calculation in the application, because they use the same members which need to be calculated correctly in previous steps
bool EnvironmentMap::addPoints(const pcl::PointCloud< pcl::PointXYZRGB>::Ptr &_cloud, const Eigen::Vector4f &_translationPrediction, const Eigen::Quaternionf &_qRotationPrediction, enum eHistoryCalculation _calculation, const double _maxFittingScore, pcl::PointCloud< pcl::PointXYZRGB>::Ptr &_addedCloud){
	switch (_calculation) {
		case eHistoryCalculation::Simple:
			return addPointsSimple(_cloud, _translationPrediction, _qRotationPrediction, _maxFittingScore, _addedCloud);
			break;
		case eHistoryCalculation::Accurate:
			return addPointsAccurate(_cloud, _maxFittingScore, _addedCloud);
			break;
		case eHistoryCalculation::Sequential:
			return addPointsSequential(_cloud, _maxFittingScore, _addedCloud);
			break;
		default:
			cout << "--> MAP: Wrong argument for addPoints()" << endl;
			return false;
			break;
	}
}

//---------------------------------------------------------------------------------------------------------------------
bool EnvironmentMap::addPointsSimple(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & _cloud, const Eigen::Vector4f &_translationPrediction, const Eigen::Quaternionf &_qRotationPrediction, const double _maxFittingScore, pcl::PointCloud< pcl::PointXYZRGB>::Ptr &_addedCloud) {
	_addedCloud = _cloud;
	if (_cloud->size() < 10) {
		std::cout << "--> MAP: addPointsSimple detects that cloud has less than 10 points, ignoring it" << std::endl;
		return false;
	}

	bool hasConverged = false;
	if (mCloud.size() == 0) {
		if (mCloudHistory.size() == 0) {
			// Store First cloud as reference
			cout << "--> MAP: This is the first point cloud, no map yet, adding to history" << endl;
			PointCloud<PointXYZRGB>::Ptr firstCloud = voxel(filter(_cloud));
			firstCloud->sensor_origin_ = Vector4f(0,0,0,1);
			firstCloud->sensor_orientation_ = Quaternionf::Identity();
			mCloudHistory.push_back(firstCloud);
			hasConverged = true;
		}
		// transform clouds to history until there are enough to make first map from history
		else if (mCloudHistory.size() < mParams.historySize) {
			cout << "--> MAP: This is point cloud Nr. " << mCloudHistory.size() + 1<< " of " << mParams.historySize << " needed for map.\n";
			hasConverged = transformCloudtoTargetCloudAndAddToHistory(_cloud, mCloudHistory[0], _maxFittingScore, transformationFromSensor(mCloudHistory.back()));
		}
	}
	else {
		// Storing and processing history of point clouds.

		Matrix4f guess = Matrix4f::Zero();
		guess.col(3) = _translationPrediction;
		guess.block<3, 3>(0, 0) = _qRotationPrediction.matrix();

		hasConverged = transformCloudtoTargetCloudAndAddToHistory(_cloud, mCloud.makeShared(), _maxFittingScore, guess);
	}

	if (hasConverged) {
		updateSensorPose(mCloudHistory.back()->sensor_origin_, mCloudHistory.back()->sensor_orientation_);
	}

	if (mCloudHistory.size() >= mParams.historySize) {
		cout << "--> MAP: Map extended" << endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr	lastJoinedCloud;
		lastJoinedCloud = convoluteCloudsInQueue(mCloudHistory).makeShared();
		mCloud += *lastJoinedCloud;
		mCloud = *voxel(mCloud.makeShared());
		// Finally discard oldest cloud
		mCloudHistory.pop_front();
		// drawing of the last point cloud that has been added to the map.
		PointCloud<PointXYZRGB> PointCloudCameraCS;
		transformPointCloud(*lastJoinedCloud, PointCloudCameraCS, originInverse(mCloud.makeShared()),sensorInverse(mCloud.makeShared()));	
		_addedCloud = PointCloudCameraCS.makeShared();

	}
	

	return hasConverged;
}
//---------------------------------------------------------------------------------------------------------------------
bool EnvironmentMap::addPointsSequential(const PointCloud<PointXYZRGB>::Ptr & _cloud, const double _maxFittingScore, pcl::PointCloud< pcl::PointXYZRGB>::Ptr &_addedCloud) {
	/*// Store First cloud as reference
	// 666 we store the actual first cloud instead of considering history, this means we accept the noise from the first cloud
	if (mCloud.size() == 0) {
		mCloud += *voxel(filter(_cloud));
		//mCloud.sensor_origin_ = Vector4f(0, 0, 0);
		//mCloud.sensor_orientation_ = Quaternionf(1, 0, 0, 0);
	}

	// Storing and processing history of point clouds.
	//temporary cleaned cloud for calculation of the transformation. We do not want to voxel in the camera coordinate system
	//becase we lose some points when rotating it to the map and voxeling there. That's why we rotate the original cloud
	PointCloud<PointXYZRGB>::Ptr filtered_cloud = filter(_cloud);
	PointCloud<PointXYZRGB> filtered_cloudWCS;
	Matrix4f transformation = getTransformationBetweenPcs(*voxel(filtered_cloud), mCloud, transformationFromSensor(mCloud.makeShared())); //666 mPreviousCloud2MapTransformation needs to be from cloudHistory.orientation
	transformPointCloud(*filtered_cloud, filtered_cloudWCS, transformation);
	PointCloud<PointXYZRGB>::Ptr voxeledFiltered_cloudWCS = voxel(filtered_cloudWCS.makeShared());
	voxeledFiltered_cloudWCS->sensor_orientation_ = Quaternionf(transformation.block<3, 3>(0, 0));
	voxeledFiltered_cloudWCS->sensor_origin_ = transformation.col(3);
	mCloudHistory.push_back(voxeledFiltered_cloudWCS);
	//666 fix this, probably not needed anymore, because it should be in mCloudHistory


	if (mCloudHistory.size() >= mParams.historySize) {
		// Get first pointcloud
		PointCloud<PointXYZRGB> convolutedSum = *mCloudHistory[0];

		for (unsigned i = 1; i < mParams.historySize; i++)
			// Now we consider only 2 clouds on history, if want to increase it, need to define points with probabilities.
			convolutedSum = convoluteCloudsOnGrid(convolutedSum, *mCloudHistory[i]);

		mCloud += convolutedSum;
		mCloud = *voxel(mCloud.makeShared());
		// Finally discart oldest cloud
		mCloudHistory.pop_front();
	}
	*/
	return false;
}

//---------------------------------------------------------------------------------------------------------------------
bool EnvironmentMap::addPointsAccurate(const PointCloud<PointXYZRGB>::Ptr & _cloud,const double _maxFittingScore,  pcl::PointCloud< pcl::PointXYZRGB>::Ptr &_addedCloud) {
	/*// Store First cloud as reference
	// 666 we store the actual first cloud instead of considering history, this means we accept the noise from the first cloud
	if (mCloud.size() == 0) {
		mCloud += *voxel(filter(_cloud));
		//mCloud.sensor_origin_ = Vector4f(0, 0, 0);
		//mCloud.sensor_orientation_ = Quaternionf(1, 0, 0, 0);
	}

	// Storing and processing history of point clouds.
	//temporary cleaned cloud for calculation of the transformation. We do not want to voxel in the camera coordinate system
	//becase we lose some points when rotating it to the map and voxeling there. That's why we rotate the original cloud
	PointCloud<PointXYZRGB>::Ptr filtered_cloud = filter(_cloud);
	PointCloud<PointXYZRGB> filtered_cloudWCS;
	Matrix4f transformation = getTransformationBetweenPcs(*voxel(filtered_cloud), mCloud, transformationFromSensor(mCloud.makeShared())); //666 mPreviousCloud2MapTransformation needs to be from cloudHistory.orientation
	transformPointCloud(*filtered_cloud, filtered_cloudWCS, transformation);
	PointCloud<PointXYZRGB>::Ptr voxeledFiltered_cloudWCS = voxel(filtered_cloudWCS.makeShared());
	voxeledFiltered_cloudWCS->sensor_orientation_ = Quaternionf(transformation.block<3, 3>(0, 0));
	voxeledFiltered_cloudWCS->sensor_origin_ = transformation.col(3);
	mCloudHistory.push_back(voxeledFiltered_cloudWCS);
	//666 fix this, probably not needed anymore, because it should be in mCloudHistory


	if (mCloudHistory.size() >= mParams.historySize) {
		// Get first pointcloud
		PointCloud<PointXYZRGB> convolutedSum = *mCloudHistory[0];

		for (unsigned i = 1; i < mParams.historySize; i++)
			// Now we consider only 2 clouds on history, if want to increase it, need to define points with probabilities.
			convolutedSum = convoluteCloudsOnGrid(convolutedSum, *mCloudHistory[i]);



		mCloud += convolutedSum;
		mCloud = *voxel(mCloud.makeShared());
		
		// Store last position of the camera.
		mCloud.sensor_orientation_ = Quaternionf(transformation.block<3, 3>(0, 0));
		mCloud.sensor_origin_ = transformation.col(3);

		// Finally discard oldest cloud
		mCloudHistory.pop_front();
	}
	*/
	return false;
}

PointCloud<PointXYZRGB>::Ptr colorizePointCloud2(const PointCloud<PointXYZRGB>::Ptr & _cloud, int _r, int _g, int _b) {
	PointCloud<PointXYZRGB>::Ptr colorizedCloud(new PointCloud<PointXYZRGB>);
	for (PointXYZRGB point : *_cloud) {
		PointXYZRGB p;
		p.x = point.x;
		p.y = point.y;
		p.z = point.z;
		p.r = _r;
		p.g = _g;
		p.b = _b;
		colorizedCloud->push_back(p);
	}
	return colorizedCloud;
}

bool EnvironmentMap::transformCloudtoTargetCloudAndAddToHistory(const PointCloud<PointXYZRGB>::Ptr & _cloud, const PointCloud<PointXYZRGB>::Ptr & _target, const double _maxFittingScore, const Matrix4f &_guess)
{
	if (_cloud->size() == 0)
		return false;

	//temporary cleaned cloud for calculation of the transformation. We do not want to voxel in the camera coordinate system
	//because we lose some points when rotating it to the map and voxeling there. That's why we rotate the original cloud
	PointCloud<PointXYZRGB>::Ptr filtered_cloud = filter(_cloud);
	
	
	Matrix4f transformation;

	PointCloud<PointXYZRGB> voxeledCloud = *voxel(filtered_cloud);
	
	pcl::PointCloud<pcl::PointXYZRGB> alignedCloud;
	bool hasConverged = getTransformationBetweenPcs(*voxel(filtered_cloud), *_target, transformation, _maxFittingScore, _guess, alignedCloud);

	bool validT = validTransformation(transformation, _guess);

	auto quatGuess = Quaternionf(_guess.block<3, 3>(0, 0));
	
	auto quatRes = Quaternionf(transformation.block<3, 3>(0, 0));
	
	mICPres = transformation;
	transformPointCloud(voxeledCloud, mGuessCloud, _guess);

	PointCloud<PointXYZRGB> filtered_cloudWCS;
	transformPointCloud(*filtered_cloud, filtered_cloudWCS, transformation);
	mAlignedCloud = *voxel(filtered_cloudWCS.makeShared());

	if (hasConverged && validT) {
		if (!hasConverged && validT) {
			cout << "--> MAP: ICP score is HIGH" << endl;
			cout << "--> MAP: we will add the cloud becuase both the translation and rotation seem good" << endl;
		}
		if (hasConverged && !validT) {
			cout << "--> MAP: This is suspicious, ICP score is good, but the transformation is not close to the guess" << endl;
			cout << "--> MAP: Possible CORRUPTED CLOUD with bad data or bad EKF prediction (if fast motion possible)" << endl;
		}
		if (hasConverged && validT) {
			cout << "--> MAP: ICP GOOD, TRANSFORMATION GOOD" << endl;
		}

		transformPointCloud(*filtered_cloud, filtered_cloudWCS, transformation);
		PointCloud<PointXYZRGB>::Ptr voxeledFiltered_cloudWCS = voxel(filtered_cloudWCS.makeShared());
	
		cout << "--> MAP: The filtered cloud has: " << filtered_cloudWCS.size() << "points" << endl;
		cout << "--> MAP: The voxeled cloud has: " << voxeledFiltered_cloudWCS->size() << "points" << endl;
		cout << "--> MAP: The guess of the transformation is" << endl << _guess << endl << "And the result is:" << endl << transformation << endl;

		voxeledFiltered_cloudWCS->sensor_orientation_ = Quaternionf(transformation.block<3, 3>(0, 0));
		voxeledFiltered_cloudWCS->sensor_origin_ = transformation.col(3);
		mCloudHistory.push_back(voxeledFiltered_cloudWCS);
		return true;
	}
	else {
		cout << "--> MAP: ICP failed, we keep EKF pose" << endl;
		return false;
	}
}

pcl::PointCloud<pcl::PointXYZRGB> EnvironmentMap::convoluteCloudsInQueue(std::deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _cloudQueue)
{
	PointCloud<PointXYZRGB> convolutedSum = *_cloudQueue[0];
	for (unsigned i = 1; i < _cloudQueue.size(); i++)
		convolutedSum = convoluteCloudsOnGrid(convolutedSum, *_cloudQueue[i]);
	return convolutedSum;
}

void EnvironmentMap::updateSensorPose(const Eigen::Vector4f &_position, const Eigen::Quaternionf &_orientation){
	mCloud.sensor_orientation_ =_orientation;
	mCloud.sensor_origin_ = _position;
}

Eigen::Vector3f EnvironmentMap::originInverse(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
	//Vector4f output = -(_cloud->sensor_orientation_.inverse()*_cloud->sensor_origin_);
	Vector3f output = -(_cloud->sensor_orientation_.inverse()*_cloud->sensor_origin_.block<3, 1>(0, 0));
	return output;
}

Eigen::Quaternionf EnvironmentMap::sensorInverse(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
	//return _cloud->sensor_orientation_.inverse();
	return _cloud->sensor_orientation_.conjugate();
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZRGB>::Ptr EnvironmentMap::voxel(const PointCloud<PointXYZRGB>::Ptr &_cloud) {
	mVoxelGrid.setInputCloud(_cloud);
	PointCloud<PointXYZRGB>::Ptr voxeled(new PointCloud<PointXYZRGB>);
	mVoxelGrid.filter(*voxeled);
	return voxeled;
}

Eigen::Matrix4f EnvironmentMap::transformationFromSensor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
	Matrix4f output = Matrix4f::Identity();
	output.col(3) = _cloud->sensor_origin_;
	output.block<3, 3>(0, 0) = _cloud->sensor_orientation_.matrix();
	return output;
}

//---------------------------------------------------------------------------------------------------------------------
vector<PointIndices> EnvironmentMap::clusterCloud(const PointCloud<PointXYZRGB>::Ptr &_cloud) {
	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
	tree->setInputCloud(_cloud);
	//vector of indices of each cluster
	vector<PointIndices> clusterIndices;
	mEuclideanClusterExtraction.setSearchMethod(tree);
	mEuclideanClusterExtraction.setInputCloud(_cloud);
	mEuclideanClusterExtraction.extract(clusterIndices);
	return clusterIndices;
}

//---------------------------------------------------------------------------------------------------------------------
vector<PointIndices> EnvironmentMap::clusterCloud(const PointCloud<PointXYZRGB>::Ptr &_cloud, vector<PointCloud<PointXYZRGB>::Ptr> &_clusters) {
	vector<PointIndices> clusterIndices = clusterCloud(_cloud);
	for (vector<PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
		PointCloud<PointXYZRGB>::Ptr cloudCluster(new PointCloud<PointXYZRGB>);
		for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			cloudCluster->points.push_back(_cloud->points[*pit]);
		}
		cloudCluster->width = cloudCluster->points.size();
		cloudCluster->height = 1;
		cloudCluster->is_dense = true;
		_clusters.push_back(cloudCluster);
	}
	return clusterIndices;
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZRGB> EnvironmentMap::cloud() {
	return mCloud;
}

//---------------------------------------------------------------------------------------------------------------------
ModelCoefficients  EnvironmentMap::extractFloor(const PointCloud<PointXYZRGB>::Ptr &_cloud) {
	vector<PointCloud<PointXYZRGB>::Ptr> clusters;
	clusterCloud(_cloud, clusters);

	if(clusters.size() < 4)
		return ModelCoefficients();

	PointCloud<PointXYZRGB> farthestPoints;
	Eigen::Vector4f pivotPt;
	pivotPt << 0,0,0,1;
	for (PointCloud<PointXYZRGB>::Ptr cluster: clusters) {
		Eigen::Vector4f maxPt;
		getMaxDistance(*cluster, pivotPt, maxPt);
		PointXYZRGB point(maxPt(0), maxPt(1), maxPt(2));
		farthestPoints.push_back(point);
	}

	ModelCoefficients::Ptr coefficients (new ModelCoefficients);
	PointIndices::Ptr inliers (new PointIndices);
	SACSegmentation<PointXYZRGB> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (SACMODEL_PLANE);
	seg.setMethodType (SAC_RANSAC);
	seg.setDistanceThreshold (0.1);
	seg.setInputCloud (farthestPoints.makeShared());
	seg.segment (*inliers, *coefficients);

	return *coefficients;
}

//---------------------------------------------------------------------------------------------------------------------
double EnvironmentMap::distanceToPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, const pcl::ModelCoefficients &_plane) {
	
	double minDist = 999999;
	for (PointXYZRGB point : *_cloud) {
		double dist =	abs(_plane.values[0]*point.x +
							_plane.values[1]*point.y +
							_plane.values[2]*point.z +
							_plane.values[3]) / 
							sqrt(	pow(_plane.values[0], 2) +
									pow(_plane.values[1], 2) +
									pow(_plane.values[2], 2));
				
		if(dist < minDist)
			minDist = dist;
	}
	
	return minDist;
}

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::cropCloud(PointCloud<PointXYZRGB>::Ptr &_cloud, ModelCoefficients _plane, bool _upperSide) {
	if (_cloud->size() == 0 || _plane.values.size() != 4)
		return;

	auto predicate = [&](const PointXYZRGB &_point) {
		double val = (-_plane.values[0] * _point.x - _plane.values[1] * _point.y - _plane.values[3])/_plane.values[2];

		if(_upperSide)
			return _point.z > val? true:false;
		else
			return _point.z > val? false:true;
	};

	_cloud->erase( std::remove_if(_cloud->begin(), _cloud->end(), predicate ), _cloud->end());
}

//---------------------------------------------------------------------------------------------------------------------
bool EnvironmentMap::getTransformationBetweenPcs(const PointCloud<PointXYZRGB>& _newCloud, const PointCloud<PointXYZRGB>& _targetCloud, Eigen::Matrix4f &_transformation,  const double _maxFittingScore, const Eigen::Matrix4f &_initialGuess, PointCloud<PointXYZRGB> &_alignedCloud) {
	mPcJoiner.setInputTarget(_targetCloud.makeShared());
	Eigen::Matrix4f prevTransformation = Eigen::Matrix4f::Identity();
	_transformation = _initialGuess;
	mPcJoiner.setMaxCorrespondenceDistance(mParams.icpMaxCorrespondenceDistance);
	bool hasConvergedInSomeIteration = false;
	int i = 0;
	for (i = 0; i < mParams.icpMaxIcpIterations; ++i){
		// Estimate
		
		mPcJoiner.setInputSource(_newCloud.makeShared());
		mPcJoiner.align(_alignedCloud, _transformation);
		//accumulate transformation between each Iteration
		_transformation = mPcJoiner.getFinalTransformation();
		if (mPcJoiner.getFinalTransformation().hasNaN()) {
			std::cout << "--> MAP: Intermedial iteration of ICP throw transformation with NaN, skiping it and continuing iterations" << std::endl;
			break;
		}
		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs((mPcJoiner.getLastIncrementalTransformation() - prevTransformation).sum()) < mPcJoiner.getTransformationEpsilon()) {
			mPcJoiner.setMaxCorrespondenceDistance(mPcJoiner.getMaxCorrespondenceDistance()*0.75);
		}

		prevTransformation = mPcJoiner.getLastIncrementalTransformation();
		hasConvergedInSomeIteration |= mPcJoiner.hasConverged();

		if (mPcJoiner.getMaxCorrespondenceDistance() < 0.001) {
			break;
		}
	}
	cout << "--> MAP: Exiting ICP iterations in " << i+1 << "/" << mParams.icpMaxIcpIterations << endl;

	mFittingScore = mPcJoiner.getFitnessScore();

	bool hasConverged = (mPcJoiner.hasConverged() || hasConvergedInSomeIteration) && mPcJoiner.getFitnessScore() < _maxFittingScore;
	if (_transformation.hasNaN()) {
		cerr << "--> MAP:  ---> CRITICAL ERROR! Transformation has nans!!! <---" << endl;
		return false;
		//cv::waitKey();
		//exit(-1);
	}
	cout << "--> MAP: Fitness score " << mPcJoiner.getFitnessScore() << "   Has conveged? " << hasConverged << endl;

	return hasConverged;
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZRGB>::Ptr colorizePointCloud(const PointCloud<PointXYZRGB>::Ptr &_cloud, int _r, int _g, int _b) {
	PointCloud<PointXYZRGB>::Ptr colorizedCloud(new PointCloud<PointXYZRGB>);
	for (PointXYZRGB point : *_cloud) {
		PointXYZRGB p;
		p.x = point.x;
		p.y = point.y;
		p.z = point.z;
		p.r = _r;
		p.g = _g;
		p.b = _b;
		colorizedCloud->push_back(p);
	}
	return colorizedCloud;
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZRGB> EnvironmentMap::convoluteCloudsOnGrid(const PointCloud<PointXYZRGB>& _cloud1, const PointCloud<PointXYZRGB>& _cloud2) {
	PointCloud<PointXYZRGB> outCloud;
	bool isFirstLarge = _cloud1.size() > _cloud2.size() ? true:false;

	// Put larger structure into VoxelGrid.
	mVoxelGrid.setSaveLeafLayout(true);
	if (isFirstLarge) 
		voxel(_cloud1.makeShared());
	else
		voxel(_cloud2.makeShared());

	// Iterate over the smaller cloud
	for (PointXYZRGB point : isFirstLarge? _cloud2 : _cloud1) {
		Eigen::Vector3i voxelCoord = mVoxelGrid.getGridCoordinates(point.x, point.y, point.z);
		int index = mVoxelGrid.getCentroidIndexAt(voxelCoord);
		if (index != -1) {
			outCloud.push_back(point);
		}
	}
	cout << "--> MAP: Size of first: " << _cloud1.size() << endl;
	cout << "--> MAP: Size of second: " << _cloud2.size() << endl;
	cout << "--> MAP: Size of result: " << outCloud.size() << endl;
	return outCloud;
}

//---------------------------------------------------------------------------------------------------------------------
bool EnvironmentMap::validTransformation(const Matrix4f & _transformation, const Matrix4f &_guess) {
	Translation3f tran = Translation3f(_guess.block<3, 1>(0, 3) - _transformation.block<3, 1>(0, 3));
	float translationChange = tran.vector().norm();
	

	Quaternionf rot = Quaternionf(_guess.block<3, 3>(0, 0));
	float angleChange = rot.angularDistance(Quaternionf(_transformation.block<3, 3>(0, 0))) * 180 / M_PI;
		
	cout << "--> MAP: ICP result is different to our provided guess by a translation of: " << endl << translationChange << endl;
	cout << "--> MAP: and a rotation of " << endl << angleChange << "deg" << endl;


	if (angleChange < mParams.icpMaxAngleChangeCompared2ProvidedGuess && translationChange < mParams.icpMaxTranslationChangeCompared2ProvidedGuess) {
		return true;
	}
	else {
		return false;
	}
}

