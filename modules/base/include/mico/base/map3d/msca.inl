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


#include <pcl/filters/random_sample.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <algorithm>    // find

namespace mico {
	//---------------------------------------------------------------------------------------------------------------------
	// Public interface
	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline bool Msca<PointType_>::compute() {
		if (mCloudQueue.size() == 0) {
			std::cout << "[MSCA] Queue is empty" << std::endl;
			return false;
		}

		mConvergedLast = false;
		unsigned iters = 0;

		// Revert transformation only if previous converged.
		/*if (mConvergedLast) {
			dealignQueue();
		}*/


		// DownSample Clouds
		VectorPointCloud sampledClouds;
		typename pcl::RandomSample<PointType_>::Ptr	cloudSampler(new pcl::RandomSample<PointType_>);
		for (auto cloud : mCloudQueue) {
			typename pcl::PointCloud<PointType_>::Ptr sampledCloud(new pcl::PointCloud<PointType_>);

			cloudSampler->setInputCloud(cloud);
			cloudSampler->setSample(cloud->size()*mSamplingFactor);
			cloudSampler->filter(*sampledCloud);

			sampledClouds.push_back(sampledCloud);

			//sampledClouds.push_back(cloud);
		}

		VectorCorrespondences correspondences;
		VectorTransformations transformations;
		VectorTransformations prevTransformations(sampledClouds.size(), Eigen::Matrix4f::Identity());
		VectorTransformations globalTransformations = prevTransformations;

		while (!mConvergedLast && iters < mMaxIters) {
			typename pcl::PointCloud<PointType_>::Ptr centerCloud(new pcl::PointCloud<PointType_>);
			// Compute correspondences between clouds
			if (!computeCorrespondences(sampledClouds, correspondences)) {
				return false;
			}

			// Compute score of currentcorres pondences
			double newScore = computeScore(correspondences);
			if (fabs(mLastScore - newScore) < mMinChangeScore) {
				mConvergedLast = true;
				break;
			}
			mLastScore = newScore;

			// Compute "centroids of correspondences" and create central cloud
			VectorPointCloud sparseCloud;
			if (!computeCentroidCloud(sampledClouds, correspondences, sparseCloud, centerCloud)) {
				return false;
			}

			// Estimate transformation between clouds using correspondences. And transform clouds
			computeTransformations(centerCloud, sparseCloud, transformations);

			// Transform sampled clouds
			for (unsigned i = 0; i < sampledClouds.size(); i++) {
				transformPointCloud(*(sampledClouds[i]), *(sampledClouds[i]), transformations[i]);
			}

			// Check convergency
			mConvergedLast = checkConvergency(transformations);

			// Update Transformations
			for (unsigned i = 0; i < globalTransformations.size(); i++) {
				globalTransformations[i] = transformations[i] * globalTransformations[i];
			}
			iters++;
		}

		if (mConvergedLast) {
			for (unsigned i = 0; i < globalTransformations.size(); i++) {
				auto inverseFirstTransformation = globalTransformations[0];
				inverseFirstTransformation.block<3, 3>(0, 0) = inverseFirstTransformation.block<3, 3>(0, 0).inverse().eval();
				inverseFirstTransformation.block<3, 1>(0, 3) = -inverseFirstTransformation.block<3, 3>(0, 0)*inverseFirstTransformation.block<3, 1>(0, 3);
				mTransformations[i] = inverseFirstTransformation*globalTransformations[i];
			}
			alignQueue();
		}

		return mConvergedLast;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline double Msca<PointType_>::score() const {
		return mLastScore;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline unsigned Msca<PointType_>::queueSize() const {
		return mCloudQueue.size();
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline const std::deque<typename pcl::PointCloud<PointType_>::Ptr> Msca<PointType_>::queue() const {
		return mCloudQueue;
	}


	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline bool Msca<PointType_>::pushCloud(pcl::PointCloud<PointType_> &cloud, double _weight) {
		mCloudQueue.push_back(cloud.makeShared());
		mWeights.push_back(_weight);
		mTransformations.push_back(Eigen::Matrix4f::Identity());
		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline pcl::PointCloud<PointType_> Msca<PointType_>::popOldest() {
		if (mCloudQueue.size() == 0)
			return pcl::PointCloud<PointType_>();
		else {
			mWeights.pop_front();
			auto poped = *mCloudQueue.front();
			mCloudQueue.pop_front();
			mTransformations.erase(mTransformations.begin());
			return poped;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline pcl::PointCloud<PointType_> Msca<PointType_>::popNewer() {
		if (mCloudQueue.size() == 0)
			return pcl::PointCloud<PointType_>();
		else {
			mWeights.pop_front();
			auto poped = *mCloudQueue.back();
			mCloudQueue.pop_back();
			mTransformations.pop_back();
			return poped;
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline void Msca<PointType_>::iterations(unsigned _iterations) {
		mMaxIters = _iterations;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline unsigned Msca<PointType_>::iterations() const {
		return mMaxIters;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline void Msca<PointType_>::maxRotation(double _max) {
		mMaxRotation = _max;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline double Msca<PointType_>::maxRotation() const {
		return mMaxRotation;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline void Msca<PointType_>::maxTranslation(double _max) {
		mMaxTranslation = _max;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline double Msca<PointType_>::maxTranslation() const {
		return mMaxTranslation;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline bool Msca<PointType_>::recomputeWeight(std::function<double(unsigned _pos, unsigned _queueSize)>) {
		return false;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline const typename Msca<PointType_>::VectorTransformations& Msca<PointType_>::transformations() const {
		return mTransformations;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline void Msca<PointType_>::indexStaticCloud(int _index) {
		mIndexStaticCloud = _index;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline void Msca<PointType_>::correspondenceDistance(double _distance) {
		mCorrespondenceDistance = _distance;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline void Msca<PointType_>::minScoreChange(double _min) {
		mMinChangeScore = _min;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline void Msca<PointType_>::samplingFactor(double _factor) {
		mSamplingFactor = _factor;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline double Msca<PointType_>::samplingFactor() {
		return mSamplingFactor;
	}


	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline void Msca<PointType_>::angleThreshold(double _angle) {
		mAngleThreshold = _angle;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline double Msca<PointType_>::angleThreshold() {
		return mAngleThreshold;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline void Msca<PointType_>::colorDistance(double &_distance) {
		mMaxColorDistance = _distance;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline double Msca<PointType_>::colorDistance() {
		return mMaxColorDistance;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline bool Msca<PointType_>::converged() {
		return mConvergedLast;
	}


	//---------------------------------------------------------------------------------------------------------------------
	// Private interface
	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline bool Msca<PointType_>::computeCorrespondences(const VectorPointCloud &_clouds, VectorCorrespondences &_correspondences) {

        unsigned referenceCloud = _clouds.size() / 2;
		_correspondences.resize(_clouds.size());
		bool someThreadFailed = false;

#pragma omp parallel
		{
#pragma omp for
			for (int i = 0; i < _clouds.size(); i++) {
				if (i != referenceCloud) {
					pcl::CorrespondencesPtr ptrCorr(new pcl::Correspondences);

					pcl::registration::CorrespondenceEstimation<PointType_, PointType_> corresp_kdtree;
					corresp_kdtree.setInputSource(_clouds[referenceCloud]);
					corresp_kdtree.setInputTarget(_clouds[i]);
                    corresp_kdtree.determineCorrespondences(*ptrCorr, mCorrespondenceDistance);


					if (ptrCorr->size() == 0) {
						someThreadFailed = true;
						std::cout << "[MSCA] Can't find any correspondences!" << std::endl;
					}
					else {
						pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rejectorNormal(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
						rejectorNormal->setThreshold(acos(mAngleThreshold*M_PI / 180.0));
						rejectorNormal->initializeDataContainer<PointType_, PointType_>();
						rejectorNormal->setInputSource<PointType_>(_clouds[referenceCloud]);
						rejectorNormal->setInputNormals<PointType_, PointType_>(_clouds[referenceCloud]);
						rejectorNormal->setInputTarget<PointType_>(_clouds[i]);
						rejectorNormal->setTargetNormals<PointType_, PointType_>(_clouds[i]);
						rejectorNormal->setInputCorrespondences(ptrCorr);
						rejectorNormal->getCorrespondences(*ptrCorr);

						pcl::CorrespondencesPtr ptrCorr2(new pcl::Correspondences);
						// Reject by color
						for (auto corr : *ptrCorr) {
							// Measure distance
							auto p1 = _clouds[referenceCloud]->at(corr.index_query);
							auto p2 = _clouds[i]->at(corr.index_match);
							double dist = sqrt(pow(p1.r - p2.r, 2) + pow(p1.g - p2.g, 2) + pow(p1.b - p2.b, 2));
							dist /= sqrt(3) * 255;

							// Add if approved
							if (dist < mMaxColorDistance) {
								ptrCorr2->push_back(corr);
							}
						}


						pcl::registration::CorrespondenceRejectorOneToOne::Ptr rejector(new pcl::registration::CorrespondenceRejectorOneToOne);

						rejector->setInputCorrespondences(ptrCorr2);
						rejector->getCorrespondences(_correspondences[i]);
					}
				}
			}
		}

		return !someThreadFailed;

	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline bool Msca<PointType_>::computeCentroidCloud(const VectorPointCloud &_clouds, const VectorCorrespondences &_correspondences, VectorPointCloud &_sparseClouds, typename pcl::PointCloud<PointType_>::Ptr &_centerCloud) {
		// Look for the element with less correspondences
        unsigned referenceCloud = _clouds.size() / 2;
		unsigned indexLessCorrespondences = 0;
		unsigned nCorrespondences = 9999999;
		for (unsigned i = 0; i < _correspondences.size(); i++) {
			if (i != referenceCloud) {
				if (_correspondences[i].size() < nCorrespondences) {
					nCorrespondences = _correspondences[i].size();
					indexLessCorrespondences = i;
				}
			}
		}

		if (nCorrespondences < 4) {
			std::cout << "[MSCA] Not enough correspondences" << std::endl;
			return false;
		}

		_sparseClouds.clear();
		for (unsigned i = 0; i < _correspondences.size(); i++) {
			_sparseClouds.push_back(typename pcl::PointCloud<PointType_>::Ptr(new pcl::PointCloud<PointType_>));
		}

		for (unsigned i = 0; i < _correspondences[indexLessCorrespondences].size(); i++) {
			auto correspondence = _correspondences[indexLessCorrespondences][i];
			bool found = true;
			PointType_ centerPoint;
			centerPoint.x = (_clouds[indexLessCorrespondences]->at(correspondence.index_match)).x		 * mWeights[indexLessCorrespondences];
			centerPoint.y = (_clouds[indexLessCorrespondences]->at(correspondence.index_match)).y		 * mWeights[indexLessCorrespondences];
			centerPoint.z = (_clouds[indexLessCorrespondences]->at(correspondence.index_match)).z		 * mWeights[indexLessCorrespondences];
			centerPoint.r = (_clouds[indexLessCorrespondences]->at(correspondence.index_match)).r		 * mWeights[indexLessCorrespondences];
			centerPoint.g = (_clouds[indexLessCorrespondences]->at(correspondence.index_match)).g		 * mWeights[indexLessCorrespondences];
			centerPoint.b = (_clouds[indexLessCorrespondences]->at(correspondence.index_match)).b		 * mWeights[indexLessCorrespondences];
			centerPoint.normal_x = (_clouds[indexLessCorrespondences]->at(correspondence.index_match)).normal_x  * mWeights[indexLessCorrespondences];
			centerPoint.normal_y = (_clouds[indexLessCorrespondences]->at(correspondence.index_match)).normal_y  * mWeights[indexLessCorrespondences];
			centerPoint.normal_z = (_clouds[indexLessCorrespondences]->at(correspondence.index_match)).normal_z  * mWeights[indexLessCorrespondences];



			centerPoint.x += (_clouds[referenceCloud]->at(correspondence.index_query)).x			*mWeights[referenceCloud];
			centerPoint.y += (_clouds[referenceCloud]->at(correspondence.index_query)).y			*mWeights[referenceCloud];
			centerPoint.z += (_clouds[referenceCloud]->at(correspondence.index_query)).z			*mWeights[referenceCloud];
			centerPoint.r += (_clouds[referenceCloud]->at(correspondence.index_query)).r			*mWeights[referenceCloud];
			centerPoint.g += (_clouds[referenceCloud]->at(correspondence.index_query)).g			*mWeights[referenceCloud];
			centerPoint.b += (_clouds[referenceCloud]->at(correspondence.index_query)).b			*mWeights[referenceCloud];
			centerPoint.normal_x += (_clouds[referenceCloud]->at(correspondence.index_query)).normal_x	*mWeights[referenceCloud];
			centerPoint.normal_y += (_clouds[referenceCloud]->at(correspondence.index_query)).normal_y	*mWeights[referenceCloud];
			centerPoint.normal_z += (_clouds[referenceCloud]->at(correspondence.index_query)).normal_z	*mWeights[referenceCloud];



			std::vector<PointType_> matchedPoints(_clouds.size());
			for (unsigned j = 0; j < _correspondences.size(); j++) {
				if (j != indexLessCorrespondences && j != referenceCloud) {
					auto iter = find_if(_correspondences[j].begin(), _correspondences[j].end(),
						[&correspondence](pcl::Correspondence _corr) {
						return correspondence.index_query == _corr.index_query;
					});
					if (iter == _correspondences[j].end()) {
						found = false;
						break;
					}
					else {
						matchedPoints[j] = _clouds[j]->at(iter->index_match);

						centerPoint.x += (_clouds[j]->at(iter->index_match)).x		*mWeights[j];
						centerPoint.y += (_clouds[j]->at(iter->index_match)).y		*mWeights[j];
						centerPoint.z += (_clouds[j]->at(iter->index_match)).z		*mWeights[j];
						centerPoint.r += (_clouds[j]->at(iter->index_match)).r		*mWeights[j];
						centerPoint.g += (_clouds[j]->at(iter->index_match)).g		*mWeights[j];
						centerPoint.b += (_clouds[j]->at(iter->index_match)).b		*mWeights[j];
						centerPoint.normal_x += (_clouds[j]->at(iter->index_match)).normal_x*mWeights[j];
						centerPoint.normal_y += (_clouds[j]->at(iter->index_match)).normal_y*mWeights[j];
						centerPoint.normal_z += (_clouds[j]->at(iter->index_match)).normal_z*mWeights[j];
					}
				}
			}

			if (found) {
				// Add points to sparse clouds for transformation estimation.
				for (unsigned j = 0; j < _correspondences.size(); j++) {
					if (j == referenceCloud) {
						_sparseClouds[j]->push_back(_clouds[j]->at(correspondence.index_query));
					}
					else if (j == indexLessCorrespondences) {
						_sparseClouds[j]->push_back(_clouds[j]->at(correspondence.index_match));
					}
					else {
						_sparseClouds[j]->push_back(matchedPoints[j]);
					}
				}

				// 
				double accWeights = accumulate(mWeights.begin(), mWeights.end(), 0);
				centerPoint.x /= accWeights;
				centerPoint.y /= accWeights;
				centerPoint.z /= accWeights;
				centerPoint.r /= accWeights;
				centerPoint.g /= accWeights;
				centerPoint.b /= accWeights;
				double norm = sqrt(centerPoint.normal_x*centerPoint.normal_x + centerPoint.normal_y*centerPoint.normal_y + centerPoint.normal_z*centerPoint.normal_z);
				centerPoint.normal_x /= norm;
				centerPoint.normal_y /= norm;
				centerPoint.normal_z /= norm;
				_centerCloud->push_back(centerPoint);
			}
		}

		if (_centerCloud->size() < 4) {
			std::cout << "[MSCA] Not enough common correspondences between clouds." << std::endl;
			return false;
		}

		return true;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline bool Msca<PointType_>::computeTransformations(const typename pcl::PointCloud<PointType_>::Ptr & _centerCloud, const VectorPointCloud &_sparseClouds, VectorTransformations& _transformations) {

		_transformations.resize(_sparseClouds.size());
		bool someThreadFailed = false;

#pragma omp parallel
		{
#pragma omp for
			for (int i = 0; i < _sparseClouds.size(); i++) {
				pcl::registration::TransformationEstimationPointToPlaneLLS<PointType_, PointType_, float> estimator;
				estimator.estimateRigidTransformation(*(_sparseClouds[i]), *_centerCloud, _transformations[i]);
				if (_transformations[i].hasNaN()) {
					std::cout << "[MSCA] Transformation of the cloud nr " << i << " contains NaN!" << std::endl;
					someThreadFailed = true;
				}
			}
		}

		return !someThreadFailed;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline double Msca<PointType_>::computeScore(VectorCorrespondences &_correspondences) {
		std::vector<double> subScores(_correspondences.size(), 0);

#pragma omp parallel
		{
#pragma omp for
			for (int i = 0; i < _correspondences.size(); i++) {
				for (unsigned j = 0; j < _correspondences[i].size(); j++) {
					subScores[i] += _correspondences[i][j].distance;
				}
			}
		}
		double res = 0;
		for (unsigned i = 0; i < subScores.size(); i++)
			res += subScores[i];

		return res;
	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline bool Msca<PointType_>::checkConvergency(const VectorTransformations&_lastTrans) {

		std::vector<int> convergences(_lastTrans.size());
#pragma omp parallel
		{
#pragma omp for
			for (int i = 0; i < _lastTrans.size(); i++) {
				Eigen::Matrix3f rot = _lastTrans[i].block<3, 3>(0, 0);
				Eigen::Vector3f angles = rot.eulerAngles(0, 1, 2);
				double rotRes = fabs(angles[0]) < M_PI ? angles[0] : angles[0] - truncf(angles[0]/M_PI)*M_PI +
								fabs(angles[1]) < M_PI ? angles[1] : angles[1] - truncf(angles[1]/M_PI)*M_PI +
								fabs(angles[2]) < M_PI ? angles[2] : angles[2] - truncf(angles[2]/M_PI)*M_PI;
				double transRes = fabs(_lastTrans[i].block<3, 1>(0, 3).sum());
				convergences[i] = (rotRes < mMaxRotation &&  transRes < mMaxTranslation) ? 0 : 1;
			}
		}

		return accumulate(convergences.begin(), convergences.end(), 0) == 0 ? true : false;

	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline void Msca<PointType_>::alignQueue() {
		for (unsigned i = 0; i < mCloudQueue.size(); i++) {
			transformPointCloud(*(mCloudQueue[i]), *(mCloudQueue[i]), mTransformations[i]);
		}

	}

	//---------------------------------------------------------------------------------------------------------------------
	template<typename PointType_>
	inline void Msca<PointType_>::dealignQueue() {
		for (unsigned i = 0; i < mCloudQueue.size(); i++) {
			auto inverseTransformation = mTransformations[i];
			inverseTransformation.block<3, 3>(0, 0) = inverseTransformation.block<3, 3>(0, 0).inverse();
			inverseTransformation.block<3, 1>(0, 3) = -inverseTransformation.block<3, 3>(0, 0)*inverseTransformation.block<3, 1>(0, 3);
			transformPointCloud(*(mCloudQueue[i]), *(mCloudQueue[i]), inverseTransformation);
		}

	}
}	//	namespace mico 
