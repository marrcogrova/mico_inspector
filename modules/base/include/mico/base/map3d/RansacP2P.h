//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

// Algorithm based on "RGB-D Mapping : Using Depth Cameras for
// Dense 3D Modeling of Indoor Environments" by Henry, Peter; 
// Krainin, Michael; Herbst, Evan; Ren, Xiaofeng; Fox, Dieter.

#ifndef _RGBDSLAM_VISION_MAP3D_RANSACP2P_H_
#define _RGBDSLAM_VISION_MAP3D_RANSACP2P_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>

#include <mico/base/map3d/DataFrame.h>

namespace mico {
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
    ///		ransac.source(source, sourceDescriptors);
    ///		ransac.target(target, targetDescriptors);
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
    template<typename PointType_>
	class RansacP2P {
	public:		// Public interface
        /// \brief Set source cloud. Source cloud will be aligned to target cloud. Descriptors will be used to aling the clouds.
        /// \param _source: cloud to be aligned to the target.
        /// \param _descriptors: descriptors of the points of cloud.
        void source(const pcl::PointCloud<PointType_> &_source, const cv::Mat &_descriptors);

        /// \brief Set target cloud. Source cloud will be aligned to target cloud. Descriptors will be used to aling the clouds.
        /// \param _target: reference cloud to align the source cloud.
        /// \param _descriptors: descriptors of the points of cloud.
        void target(const pcl::PointCloud<PointType_> &_target, const cv::Mat &_descriptors);

        /// \brief Set source cloud. Source cloud will be aligned to target cloud. By this way, not matches are computed between the clouds, it is taken from outside using the @_matches argument
        /// \param _source: cloud to be aligned to the target.
        /// \param reference cloud to align the source cloud.
        /// \param _matches matches between the source cloud to the target cloud
        void sourceTarget(const pcl::PointCloud<PointType_> &_source, const pcl::PointCloud<PointType_> &_target, const std::vector<cv::DMatch> &_matches);

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

        /// \brief set min number of inliers needed for accepting the cloud
        void minInliers(int _min);

        /// \brief get min number of inliers needed for accepting the cloud
        int minInliers();

        /// \brief Get inliers of previous execution of the algorithm.
        /// \params _inliers: vector containing the indices of points in source cloud that are inliers.
        void inliers(std::vector<int> &_inliers);

        /// \brief Get inliers of previous execution of the algorithm.
        /// \params _inliers: vector containing the matches considered inliers;
        void inliers(std::vector<cv::DMatch> &_inliers);

		/// \brief Get score of the last execution of the algorithm.
		double score();

		/// \brief Run algorithm. Returns true if executed properly, false if not.
		bool run();

        /// \brief get best transformation obtained.
        Eigen::Matrix4f transformation();

        DataFrame<PointType_> srcKf, tgtKf;

	private:	// Members
        typename pcl::PointCloud<PointType_> mSource, mTarget;
		cv::Mat mSourceDescriptors, mTargetDescriptors;
        std::vector<cv::DMatch> mMatches;

		unsigned mMaxIters = 100;
        double mMaxDistance = 0.01;
        int mMinInliers = 8;

        std::vector<int> mInliers;
        std::vector<cv::DMatch> mInliersMatches;
        Eigen::Matrix4f mLastTransformation;
		double mLastScore = INFINITY;

	};
}	//	namespace mico 


#include "RansacP2P.inl"


#endif	//	_RGBDSLAM_VISION_MAP3D_RANSACP2P_H_
