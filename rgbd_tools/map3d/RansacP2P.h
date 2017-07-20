////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////
// Algorithm based on "RGB-D Mapping : Using Depth Cameras for 
// Dense 3D Modeling of Indoor Environments" by Henry, Peter; 
// Krainin, Michael; Herbst, Evan; Ren, Xiaofeng; Fox, Dieter.

#ifndef _RGBDSLAM_VISION_MAP3D_RANSACP2P_H_
#define _RGBDSLAM_VISION_MAP3D_RANSACP2P_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>

namespace rgbd {
	/// \brief Point to point ransac implementation for fast cloud registration. 
	///	Example of use:
	///
	///	\code
	///		// Initialize clouds
	///		PointCloud<PointXYZ> source = fillSourceCloud();
	///		PointCloud<PointXYZ> target = fillTargetCloud();
	///		
	///		// Configure ransac
	///		RansacP2P ransac;	
	///		ransac.source(source);
	///		ransac.target(target);
	///		ransac.maxIters(160);		// Set number of samples
	///		ransac.maxDistance(0.05);	// Set maximum distance between points to consider them outliers.
	///		
	///		// Run it
	///		ransac.run()
	///		
	///		// Get results
	///		double finalScore = ransac.score();
	///		std::vector<int> inliers;
	///		ransac.inliers(inliers);
	///
	///	\endcode
	///
	class RansacP2P {
	public:		// Public interface
		/// \brief Set source cloud. Source cloud will be aligned to target cloud.
		/// \param _source: cloud to be aligned to the target.
		void source(const pcl::PointCloud<pcl::PointXYZ> &_source, const cv::Mat &_descriptors);

		/// \brief Set target cloud. Source cloud will be aligned to target cloud.
		/// \param _target: reference cloud to align the source cloud.
		void target(const pcl::PointCloud<pcl::PointXYZ> &_target, const cv::Mat &_descriptors);

		/// \brief Set number of max iterations. (Default = 100).
		/// \param _iters: desired number of max iterations
		void maxIters(unsigned _iters);

		/// \brief Get max iterations allowed.
		unsigned maxIters();

		/// Set max distance allowed to reject outliers. (Default = 0.01).
		/// \param _dist: maximum allowed distance.
		void maxDistance(double _distance);

		/// \brief Get max allowed distance to reject outliers
		double maxDistance();

		/// \brief Get inliers of previous execution of the algorithm.
		/// \params _inliers: vector containing the indices of points in source cloud that are inliers.
		void inliers(std::vector<int> &_inliers);

		/// \brief Get score of the last execution of the algorithm.
		double score();

		/// \brief Run algorithm. Returns true if executed properly, false if not.
		bool run();

	private:	// Private methods


	private:	// Members
		pcl::PointCloud<pcl::PointXYZ>::Ptr mSource, mTarget;
		cv::Mat mSourceDescriptors, mTargetDescriptors;

		unsigned mMaxIters = 100;
		double mMaxSquaredDistance = 0.01*0.01;

		std::vector<int> mInliers;
		double mLastScore = INFINITY;

	};
}	//	namespace rgbd

#endif	//	_RGBDSLAM_VISION_MAP3D_RANSACP2P_H_