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


#ifndef MICO_BASE_MAP3D_MSCA_H_
#define MICO_BASE_MAP3D_MSCA_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include <vector>
#include <deque>
#include <functional>

#include<Eigen/Eigen>

namespace mico {
	/// Class for Multiple Simultaneous Cloud Alignment. Each iteration, it is computed a cloud positioned in the center of all clouds,
	///	then transformation between each cloud and the central one is computed. This process is repeated until convergence or a number o
	///	of iterations.
	///
	///	Example of usage:
	///
	///	\code
	///		// Create clouds
	///		auto cloud1 = getCloud1();
	///		auto cloud2 = getCloud2();
	///		auto cloud3 = getCloud3();
	///
	///		// Init MSCA
	///		Msca msca;
	///		msca.iterations(20);
	///		msca.maxTransformation(1e-10);
	///
	///		// Push clouds into queue
	///		msca.pushCloud(cloud1);
	///		msca.pushCloud(cloud2);
	///		msca.pushCloud(cloud3);
	///
	///		// Compute transformations
	///		if(msca.compute()){
	///			msca.popNewest();	// Discart wrong newest cloud
	///		} else {
	///			auto cloodForMap = msca.popOldest();
	///			// do whatever.
	///		}
	///
	///		// Get a new cloud and so on...
	///		auto cloud4 = getCloud4();
	///		msca.pushCloud(cloud4);
	///
	///		// Compute again
	///		msca.compute();
	///
	///	\endcode
	///

	template<typename PointType_>
	class Msca {
	public:	// Public interface
		typedef std::vector<typename pcl::PointCloud<PointType_>::Ptr, Eigen::aligned_allocator<typename pcl::PointCloud<PointType_>::Ptr> > VectorPointCloud;
		typedef std::vector<pcl::Correspondences, Eigen::aligned_allocator<pcl::Correspondences>> VectorCorrespondences;
		typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> VectorTransformations;

		/// Compute optimization in all the clouds in the queue.
		bool compute();

		/// Get last score.
		double score() const;

		/// Get number of elements in the queue.
		unsigned queueSize() const;

		/// Get queue.
		const std::deque<typename pcl::PointCloud<PointType_>::Ptr> queue() const;

		/// Add a new cloud into the queue.
		bool pushCloud(pcl::PointCloud<PointType_> &cloud, double _weight = 1.0);

		/// Remove cloud in the front of the queue.
		pcl::PointCloud<PointType_> popOldest();

		/// Remove latest cloud (cloud in the back) added into the queue.
		pcl::PointCloud<PointType_> popNewer();

		/// Set maximum number of iterations. Defaul 10.
		void iterations(unsigned _iterations);

		/// Get maximum number of iterations. Defaul 10.
		unsigned iterations() const;

		/// Set maximum rotation allowed between clouds (max sum of euler angles). Default 0.02.
		void maxRotation(double _max);

		/// Get maximum rotation allowed vetween clouds (max sum of euler angles). Default 0.02.
		double maxRotation() const;

		/// Set maximum translation allowed between clouds. Default 0.001.
		void maxTranslation(double _max);

		/// Get maximum translation allowed vetween clouds. Default 0.001.
		double maxTranslation() const;

		/// Recompute weigths using their position in the queue and using the given function.
		bool recomputeWeight(std::function<double(unsigned _pos, unsigned _queueSize)>);

		/// Get last transformations to align cloud. This transformation align them between each other not to any referente frame.
		const VectorTransformations & transformations() const;

		/// Set an index of a cloud to make it remain static. Default -1 (no one) [DEPRECATED]
		void indexStaticCloud(int _index);

		/// Set range for searching correspondences	(Default 0.3)
		/// \param _distance: maximun allowed distance to look for correspondences in clouds.
		void correspondenceDistance(double _distance);

		/// Set minimum change in score for convergence criteria. If score changes less than that minimum value
		/// between two iterations, then algorithm is considered converged. (Default 1e-5).
		/// \param _minScore: threshold for score change.
		void minScoreChange(double _min);

		/// Set the sampling factor. Sampling factor is basically the proportion of points that we want to use for the
		/// alignment procedure. These points are chosen randomly using a random sampling filter. (Default 0.25)
		///	\param _factor: porportion of points to be used.
		void samplingFactor(double _factor);

		/// Set the sampling factor. Sampling factor is basically the proportion of points that we want to use for the
		/// alignment procedure. These points are chosen randomly using a random sampling filter. (Default 0.25)
		/// \return proportion of points being used for the alignment procedure.
		double samplingFactor();

		/// Set maximum angle allowed for normals to differ in degrees.
		void angleThreshold(double _angle);

		/// get maximum angle allowed for normals to differ in degrees.
		double angleThreshold();

		/// Set maximum distance between colors for correspondence rejection. Normalized within 0 and 1.
		void colorDistance(double &_distance);

		/// Get maximum distance between colors for correspondence rejection. Normalized within 0 and 1.
		double colorDistance();

		/// Return true if the previous alignment converged
		bool converged();

	private:	// Private interface
		bool computeCorrespondences(const VectorPointCloud &_clouds, VectorCorrespondences&_correspondences);
		bool computeCentroidCloud(const VectorPointCloud &_clouds, const VectorCorrespondences &_correspondences, VectorPointCloud &_sparseClouds, typename pcl::PointCloud<PointType_>::Ptr &_centerCloud);
		bool computeTransformations(const typename pcl::PointCloud<PointType_>::Ptr & _centerCloud, const VectorPointCloud & _sparseClouds, VectorTransformations& _transformations);
		double computeScore(VectorCorrespondences &_correspondences);
		bool checkConvergency(const VectorTransformations&_lastTrans);


		void alignQueue();
		void dealignQueue();

	private:	// Members
		std::deque<typename pcl::PointCloud<PointType_>::Ptr>	mCloudQueue;
		std::deque<double>										mWeights;

		VectorTransformations	mTransformations;
		double					mLastScore;
		bool					mConvergedLast;

		double		mMinChangeScore = 1e-5;
		double		mCorrespondenceDistance = 0.3;
		double		mMaxRotation = 0.02;
		double		mMaxTranslation = 0.001;
		unsigned	mMaxIters = 10;
		int			mIndexStaticCloud = -1;
		double		mSamplingFactor = 0.25;
		double		mAngleThreshold = 45;
		double		mMaxColorDistance = 0.3;
	};
}	//	namespace mico 

#include "msca.inl"

#endif	//	RGBDSLAM_MAP3D_MSCA_H_

