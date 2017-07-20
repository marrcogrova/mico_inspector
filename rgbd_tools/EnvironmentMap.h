////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#ifndef RGBDSLAM_VISION_ENVIRONMENTMAP_H_
#define RGBDSLAM_VISION_ENVIRONMENTMAP_H_

#include <unordered_map>
#include <cjson/json.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map3d/msca.h>

namespace rgbd {
	class EnvironmentMap {
	public:		// Public interface
		typedef pcl::PointXYZRGBNormal PointType;
		typedef pcl::FPFHSignature33 FeatureType;


		/// \brief Initialize class using the given json cnfig file
		///	Example of Json file.
		///	@code
		///		{
		///			"msca":
		///				{
		///                 "iterations":50,
		///                 "minScoreChange":0.000001,
		///                 "maxRotation":0.002,
		///                 "maxTranslation":0.001,
		///                 "correspondenceDistance":0.3,
		///                 "indexStaticCloud":0,
		///                 "samplingFactor":0.5,
		///					"queueSize":2
		///				},
		///			"voxelSize":0.001
		///		}
		///	@endcode

		bool init(const cjson::Json &_configFile);

		/// \brief get current dense map.
		pcl::PointCloud<PointType> map() const;

		/// \brief update map with new data. Only use ICP to align cloud. This method assume dense clouds to compute normals.
		/// \param _cloud: new dense cloud
		/// \param _guess: guess for cloud alignment
		bool update(pcl::PointCloud<PointType>::Ptr _cloud, Eigen::Matrix4f &_guess);

		/// \brief update map with new data. Only use ICP to align cloud. This method assume dense clouds to compute normals.
		/// \param _cloud: new dense cloud
		bool update(pcl::PointCloud<PointType>::Ptr _cloud);

	private:	// Private methods
		bool alignAndMerge();

	private:	// Members
		std::unordered_map<std::string, double>			mParameters;
		pcl::PointCloud<PointType>::Ptr	mDenseMap, mLastCloud;
		double mVoxelSize;
		unsigned mQueueSize;
		Msca<PointType> mMsca;
	};	//	class EnvironmentMap


	Eigen::Matrix3f rotVector2Matrix(const double &_rx, const double &_ry, const double &_rz);
}	//	namespace rgbd

#endif	//	RGBDSLAM_VISION_ENVIRONMENTMAP_H_
