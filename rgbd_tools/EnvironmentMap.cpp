////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include "EnvironmentMap.h"

#include <chrono>
#include <cassert>

#include <utils/Gui.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;
using namespace std;
using namespace pcl::registration;

namespace rgbd {

	bool EnvironmentMap::init(const cjson::Json &_config) {
		while (mMsca.queueSize() != 0) {
			mMsca.popOldest();
		}
		// Set up the feature detector
		mMsca.minScoreChange((float)_config["msca"]["minScoreChange"]);
		mMsca.iterations((int)_config["msca"]["iterations"]);
		mMsca.maxRotation((float)_config["msca"]["maxRotation"]);
		mMsca.maxTranslation((float)_config["msca"]["maxTranslation"]);
		mMsca.correspondenceDistance((float)_config["msca"]["correspondenceDistance"]);
		mMsca.indexStaticCloud((int)_config["msca"]["indexStaticCloud"]);
		mMsca.samplingFactor((float)_config["msca"]["samplingFactor"]);
		mMsca.angleThreshold((float)_config["msca"]["angleThreshold"]);

		mDenseMap = PointCloud<PointType>::Ptr(new PointCloud<PointType>());
		mVoxelSize = (float)_config["voxelSize"];
		mQueueSize = (int)_config["msca"]["queueSize"];
		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	PointCloud<PointXYZRGBNormal> EnvironmentMap::map() const {
		return *mDenseMap;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool EnvironmentMap::update(pcl::PointCloud<PointType>::Ptr _cloud, Eigen::Matrix4f &_guess) {
		transformPointCloud(*_cloud, *_cloud, _guess);

		if (mMsca.queueSize() < mQueueSize - 1) {
			mMsca.pushCloud(*_cloud);
		}
		else {
			mMsca.pushCloud(*_cloud);
			alignAndMerge();
		}
		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool EnvironmentMap::update(pcl::PointCloud<PointType>::Ptr _cloud) {
		auto trans = mMsca.transformations();
		if (trans.size() != 0) {
			transformPointCloud(*_cloud, *_cloud, trans.back());
		}

		if (mMsca.queueSize() < mQueueSize - 1) {
			mMsca.pushCloud(*_cloud);
		}
		else {
			mMsca.pushCloud(*_cloud);

			alignAndMerge();
		}
		return true;
	}
	//---------------------------------------------------------------------------------------------------------------------
	// Provate methods
	//---------------------------------------------------------------------------------------------------------------------

	bool EnvironmentMap::alignAndMerge() {
		if (mMsca.compute()) {
			auto newCloud = mMsca.popOldest();
			*mDenseMap += newCloud;

			PointCloud<PointXYZRGBNormal> map;
			pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
			sor.setInputCloud(mDenseMap);
			sor.setLeafSize(mVoxelSize, mVoxelSize, mVoxelSize);
			sor.filter(map);
			mDenseMap = map.makeShared();
			return true;
		}
		else {
			std::cout << "MSCA didn't converge" << std::endl;
			mMsca.popNewer();
			return false;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	// Other functions
	//---------------------------------------------------------------------------------------------------------------------
	Eigen::Matrix3f rotVector2Matrix(const double &_rx, const double &_ry, const double &_rz) {
		Eigen::Vector3f v(_rx, _ry, _rz);
		float norm = v.norm();

		Eigen::Matrix3f rotMat;

		float sv2 = sin(norm / 2);
		float cv2 = cos(norm / 2);

		rotMat(0, 0) = ((_rx*_rx - _ry*_ry - _rz*_rz)*sv2*sv2 + norm*norm*cv2*cv2) / norm / norm;
		rotMat(0, 1) = (2 * sv2*(_rx*_ry*sv2 - norm*_rz*cv2)) / norm / norm;
		rotMat(0, 2) = (2 * sv2*(_rx*_rz*sv2 + norm*_ry*cv2)) / norm / norm;

		rotMat(1, 0) = (2 * sv2*(_rx*_ry*sv2 + norm*_rz*cv2)) / norm / norm;
		rotMat(1, 1) = ((_ry*_ry - _rx*_rx - _rz*_rz)*sv2*sv2 + norm*norm*cv2*cv2) / norm / norm;
		rotMat(1, 2) = (2 * sv2*(_ry*_rz*sv2 - norm*_rx*cv2)) / norm / norm;

		rotMat(2, 0) = (2 * sv2*(_rx*_rz*sv2 - norm*_ry*cv2)) / norm / norm;
		rotMat(2, 1) = (2 * sv2*(_ry*_rz*sv2 + norm*_rx*cv2)) / norm / norm;
		rotMat(2, 2) = ((_rz*_rz - _ry*_ry - _rx*_rx)*sv2*sv2 + norm*norm*cv2*cv2) / norm / norm;


		return rotMat;

	}
}	//	namespace rgbd
