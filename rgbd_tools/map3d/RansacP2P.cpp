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

#include "RansacP2P.h"

#include <time.h>	// 666 TODO clean up if needed.
#include <random>

#include <opencv2/opencv.hpp>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace cv;
using namespace pcl;
using namespace std;

namespace rgbd {
	//---------------------------------------------------------------------------------------------------------------------
	void RansacP2P::source(const PointCloud<PointXYZ>& _source, const cv::Mat &_descriptors) {
		mSource = _source.makeShared();
		mSourceDescriptors = _descriptors;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void RansacP2P::target(const PointCloud<PointXYZ>& _target, const cv::Mat &_descriptors) {
		mTarget = _target.makeShared();
		mTargetDescriptors = _descriptors;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void RansacP2P::maxIters(unsigned _iters) {
		mMaxIters = _iters;
	}

	//---------------------------------------------------------------------------------------------------------------------
	unsigned RansacP2P::maxIters() {
		return mMaxIters;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void RansacP2P::maxDistance(double _distance) {
		mMaxSquaredDistance = _distance*_distance;
	}

	//---------------------------------------------------------------------------------------------------------------------
	double RansacP2P::maxDistance() {
		return sqrt(mMaxSquaredDistance);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void RansacP2P::inliers(vector<int>& _inliers) {
		_inliers = mInliers;
	}

	//---------------------------------------------------------------------------------------------------------------------
	double RansacP2P::score() {
		return mMaxIters;
	}

	//---------------------------------------------------------------------------------------------------------------------
	bool RansacP2P::run() {
		// Check arguments
		if (!mSource || !mTarget || mSourceDescriptors.rows == 0 || mTargetDescriptors.rows == 0) {
			cout << "[RANSACP2P] Error, some argument missing" << endl;
			return false;
		}

		srand(time(NULL));	// 666 TODO if it takes long time, move to another place!
		random_device rd;
		mt19937 gen(rd());
		uniform_int_distribution<> dis(0, mMaxIters);

		// Compute matches between point clouds.
		FlannBasedMatcher matcher;
		vector<DMatch> matches;
		matcher.match(mSourceDescriptors, mTargetDescriptors, matches);

		vector<Eigen::Matrix4f> transformations(mMaxIters);
		vector<double> scores(mMaxIters, 0);
		vector<vector<int>> inliers(mMaxIters);

		// Sample points.
		//#pragma omp parallel for		
		for (int i = 0; i < mMaxIters; i++) {
			// int i = omp_get_thread_num();
			// Take 3 random matches.
			PointCloud<PointXYZ> src, tgt;
			for (unsigned pi = 0; pi < 3; pi++) {
				DMatch rndMatch = matches[dis(gen)];
				src.push_back(mSource->at(rndMatch.queryIdx));
				tgt.push_back(mTarget->at(rndMatch.trainIdx));
			}

			// Compute rigid transform. Using Umeyama estimator
			pcl::registration::TransformationEstimationSVD<PointXYZ, PointXYZ, float> transformationEstimator;
            transformationEstimator.estimateRigidTransformation(src, tgt, transformations[i]);

			// Rotate cloud
			PointCloud<PointXYZ> transformedCloud;
            transformPointCloud(*mSource, transformedCloud, transformations[i]);

			// Reject outliers, compute cost and store result.
			for (unsigned pi = 0; pi < transformedCloud.size(); pi++) {
				PointXYZ pSrc = transformedCloud[pi];
				for (PointXYZ pTgt : *mTarget) {
					float diffX = pTgt.x - pSrc.x, diffY = pTgt.y - pSrc.y, diffZ = pTgt.z - pSrc.z;
					double squaredDist = diffX*diffX + diffY*diffY + diffZ*diffZ;

					if (squaredDist < mMaxSquaredDistance) {	// Inlier
                        inliers[i].push_back(pi);
                        scores[i] += squaredDist;
					}
					else {	// Outlier
						// Nothing to do.
					}
				}
			}
			// Normalize score.
            scores[i] /= inliers[i].size();

		} // End omp parallel for

		// Choose best option.
		double minScore = 9999999;
		unsigned minIndex = 0;
		for (unsigned i = 0; i < scores.size(); i++) {
			if (scores[i] < minScore) {
				minScore = scores[i];
				minIndex = i;
			}
		}

		mInliers = inliers[minIndex];
		mLastScore = scores[minIndex];

		return true;
	}
}	//	namespace rgbd
