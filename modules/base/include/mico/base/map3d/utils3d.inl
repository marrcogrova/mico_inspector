//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92)
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


#include <Eigen/Eigen>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <chrono>

namespace mico {
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    void ransacAlignment(typename pcl::PointCloud<PointType_>::Ptr _source,
                         typename pcl::PointCloud<PointType_>::Ptr _target,
                         std::vector<cv::DMatch> &_matches,
                         Eigen::Matrix4f &_transformation,
                         std::vector<int> &_inliers,
                         double _maxRansacDistance,
                         int _ransacIterations,
                         unsigned _refineIterations) {

        std::vector<int> source_indices (_matches.size());   // 666 matchesPrev should be removed from kfs struct.
        std::vector<int> target_indices (_matches.size());

        // Copy the query-match indices
        for (int i = 0; i < (int)_matches.size(); ++i) {
            source_indices[i] = _matches[i].queryIdx;
            target_indices[i] = _matches[i].trainIdx;
        }

        typename pcl::SampleConsensusModelRegistration<PointType_>::Ptr model(new pcl::SampleConsensusModelRegistration<PointType_>(_source, source_indices));

        // Pass the target_indices
        model->setInputTarget (_target, target_indices);

        // Create a RANSAC model
        pcl::RandomSampleConsensus<PointType_> sac (model, _maxRansacDistance);

        sac.setMaxIterations(_ransacIterations);
        // Compute the set of inliers
        if(sac.computeModel()) {
            Eigen::VectorXf model_coefficients;

            sac.getInliers(_inliers);

            sac.getModelCoefficients (model_coefficients);

            // REFINEMENT STEP
            if (_refineIterations > 0) {
                double error_threshold = _maxRansacDistance;
                unsigned int refine_iterations = 0;
                bool inlier_changed = false;
                [[maybe_unused]] bool oscillating = false;
                std::vector<int> new_inliers, prev_inliers = _inliers;
                std::vector<size_t> inliers_sizes;
                Eigen::VectorXf new_model_coefficients = model_coefficients;
                do {
                    // Optimize the model coefficients
                    model->optimizeModelCoefficients (prev_inliers, new_model_coefficients, new_model_coefficients);
                    inliers_sizes.push_back (prev_inliers.size ());

                    // Select the new inliers based on the optimized coefficients and new threshold
                    model->selectWithinDistance (new_model_coefficients, error_threshold, new_inliers);

                    if (new_inliers.empty ()) {
                        ++refine_iterations;
                        if (refine_iterations >= _refineIterations) {
                            break;
                        }
                        continue;
                    }

                    // Estimate the variance and the new threshold
                    double variance = model->computeVariance ();
                    double refineSigma = 3.0;
                    error_threshold = std::min (_maxRansacDistance, refineSigma * sqrt(variance));

                    inlier_changed = false;
                    std::swap (prev_inliers, new_inliers);

                    // If the number of inliers changed, then we are still optimizing
                    if (new_inliers.size () != prev_inliers.size ()) {
                        // Check if the number of inliers is oscillating in between two values
                        if (inliers_sizes.size () >= 4) {
                            if (inliers_sizes[inliers_sizes.size () - 1] == inliers_sizes[inliers_sizes.size () - 3] &&
                            inliers_sizes[inliers_sizes.size () - 2] == inliers_sizes[inliers_sizes.size () - 4]) {
                                oscillating = true;
                                break;
                            }
                        }
                        inlier_changed = true;
                        continue;
                    }

                    // Check the values of the inlier set
                    for (size_t i = 0; i < prev_inliers.size (); ++i) {
                        // If the value of the inliers changed, then we are still optimizing
                        if (prev_inliers[i] != new_inliers[i]){
                            inlier_changed = true;
                            break;
                        }
                    }
                } while (inlier_changed && ++refine_iterations < _refineIterations);

                std::swap (_inliers, new_inliers);
                model_coefficients = new_model_coefficients;
            }


            [[maybe_unused]] double covariance = model->computeVariance();
            // get best transformation
            Eigen::Matrix4f bestTransformation;
            bestTransformation.row (0) = model_coefficients.segment<4>(0);
            bestTransformation.row (1) = model_coefficients.segment<4>(4);
            bestTransformation.row (2) = model_coefficients.segment<4>(8);
            bestTransformation.row (3) = model_coefficients.segment<4>(12);
            _transformation = bestTransformation;
        }
    }

    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    bool icpAlignment(typename pcl::PointCloud<PointType_>::Ptr _source,
                      typename pcl::PointCloud<PointType_>::Ptr _target,
                      Eigen::Matrix4f &_transformation,
                      int _iterations,
                      double _correspondenceDistance,
                      double _maxAngleDistance,
                      double _maxColorDistance,
                      double _maxTranslation,
                      double _maxRotation,
                      double _maxFitnessScore,
                      double _voxelGridSize,
                      double _timeout) {
        LoggableInterface<DebugLevel_, OutInterface_> logDealer;

        auto t0 = std::chrono::high_resolution_clock::now();
        double timeSpent = 0;
        pcl::PointCloud<PointType_> srcCloud;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*_source, srcCloud, indices);
        pcl::PointCloud<PointType_> tgtCloud;
        pcl::removeNaNFromPointCloud(*_target, tgtCloud, indices);

        pcl::VoxelGrid<PointType_> sor;
        sor.setLeafSize (_voxelGridSize,_voxelGridSize,_voxelGridSize);
        sor.setInputCloud (srcCloud.makeShared());
        sor.filter (srcCloud);
        sor.setInputCloud (tgtCloud.makeShared());
        sor.filter (tgtCloud);

        bool converged = false;
        int iters = 0;
        double corrDistance = _correspondenceDistance;
        while (/*!converged &&*/ iters < _iterations && timeSpent < _timeout) {   
            iters++;
            pcl::PointCloud<PointType_> cloudToAlign;
            //std::cout << _transformation << std::endl;
            pcl::transformPointCloudWithNormals(srcCloud, cloudToAlign, _transformation);
            
            // COMPUTE CORRESPONDENCES
            pcl::Correspondences correspondences;
            pcl::CorrespondencesPtr ptrCorr(new pcl::Correspondences);

            pcl::registration::CorrespondenceEstimation<PointType_, PointType_> corresp_kdtree;
            corresp_kdtree.setInputSource(cloudToAlign.makeShared());
            corresp_kdtree.setInputTarget(tgtCloud.makeShared());
            corresp_kdtree.determineCorrespondences(*ptrCorr, corrDistance);

            //std::cout << "Found " << ptrCorr->size() << " correspondences by distance" << std::endl;

            if (ptrCorr->size() == 0) {
                logDealer.error("ICP_ALIGNEMENT", "Can't find any correspondence");
                //std::cout << "Can't find any correspondences!" << std::endl;
                break;
            }
            else {
                pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rejectorNormal(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
                rejectorNormal->setThreshold(_maxAngleDistance);
                rejectorNormal->initializeDataContainer<PointType_, PointType_>();
                rejectorNormal->setInputSource<PointType_>(cloudToAlign.makeShared());
                rejectorNormal->setInputNormals<PointType_, PointType_>(cloudToAlign.makeShared());
                rejectorNormal->setInputTarget<PointType_>(tgtCloud.makeShared());
                rejectorNormal->setTargetNormals<PointType_, PointType_>(tgtCloud.makeShared());
                rejectorNormal->setInputCorrespondences(ptrCorr);
                rejectorNormal->getCorrespondences(*ptrCorr);
                //std::cout << "Found " << ptrCorr->size() << " correspondences after normal rejection" << std::endl;

                pcl::registration::CorrespondenceRejectorOneToOne::Ptr rejector(new pcl::registration::CorrespondenceRejectorOneToOne);
                rejector->setInputCorrespondences(ptrCorr);
                rejector->getCorrespondences(*ptrCorr);
                //std::cout << "Found " << ptrCorr->size() << " correspondences after one to one rejection" << std::endl;

                //pcl::CorrespondencesPtr ptrCorr2(new pcl::Correspondences);
                if(fabs(_maxColorDistance - 1)  < 0.01){    // 666 Just in case
                    // Reject by color
                    for (auto corr : *ptrCorr) {
                        // Measure distance
                        auto p1 = cloudToAlign.at(corr.index_query);
                        auto p2 = tgtCloud.at(corr.index_match);
                        double dist = sqrt(pow(p1.r - p2.r, 2) + pow(p1.g - p2.g, 2) + pow(p1.b - p2.b, 2));
                        dist /= sqrt(3) * 255;

                        // Add if approved
                        if (dist < _maxColorDistance) {
                            correspondences.push_back(corr);
                        }
                    }
                }else{
                    correspondences = *ptrCorr;
                }

                //std::cout << "Found " << correspondences.size() << " correspondences after color rejection" << std::endl;
            }

            // Estimate transform
            pcl::registration::TransformationEstimationPointToPlaneLLS<PointType_, PointType_, float> estimator;
            //std::cout << _transformation << std::endl;
            Eigen::Matrix4f incTransform;
            estimator.estimateRigidTransformation(cloudToAlign, tgtCloud, correspondences,  incTransform);
            if (incTransform.hasNaN()) {
                logDealer.error("ICP_ALIGNEMENT", "Transformation of the cloud contains NaN");
                continue;
            }

            // COMPUTE SCORE
            double score = 0;
            for (unsigned j = 0; j < correspondences.size(); j++) {
                score += correspondences[j].distance;
            }

            // CONVERGENCE
            Eigen::Matrix3f rot = incTransform.block<3, 3>(0, 0);
            Eigen::Quaternionf q(rot);
            Eigen::Quaternionf q0(Eigen::Matrix3f::Identity());
            double rotRes = fabs(q0.x() - q.x())+fabs(q0.z() - q.z())+fabs(q0.y() - q.y())+fabs(q0.w() - q.w());
            double transRes = fabs(incTransform.block<3, 1>(0, 3).sum());
            converged = (rotRes < _maxRotation &&  transRes < _maxTranslation) ? 1 : 0;

            //std::cout << "incT: " << transRes << ". incR: " << rotRes << ". Score: " << score << std::endl;
            converged = converged && (score < _maxFitnessScore);
            _transformation = incTransform*_transformation;
            corrDistance *= 0.9;     

            auto t1 = std::chrono::high_resolution_clock::now();
            timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
        }

        return converged;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    bool  transformationBetweenFeatures(std::shared_ptr<DataFrame<PointType_>> &_previousKf,
                                        std::shared_ptr<DataFrame<PointType_>> &_currentKf,
                                        Eigen::Matrix4f &_transformation,
                                        double _mk_nearest_neighbors,
                                        double _mRansacMaxDistance,
                                        int _mRansacIterations,
                                        double _mRansacMinInliers,
                                        double _mFactorDescriptorDistance,
                                        unsigned _mRansacRefineIterations){

        LoggableInterface<DebugLevel_, OutInterface_> logDealer;
        
        if(_currentKf->multimatchesInliersKfs.find(_previousKf->id) !=  _currentKf->multimatchesInliersKfs.end()){
            // Match already computed
            logDealer.status("TRANSFORM_BETWEEN_FEATURES",  "Match already computed between frames: " + 
                                                            std::to_string(_currentKf->id) + " and " + 
                                                            std::to_string(_previousKf->id));
            return true;
        }
        std::vector<cv::DMatch> matches;
        matchDescriptors(   _currentKf->featureDescriptors,
                            _previousKf->featureDescriptors,
                            matches,
                            _mk_nearest_neighbors,
                            _mFactorDescriptorDistance);
        
        std::vector<int> inliers;
        if(_mk_nearest_neighbors>1){
            typename pcl::PointCloud<PointType_>::Ptr duplicateCurrentKfFeatureCloud = _currentKf->featureCloud;
            *duplicateCurrentKfFeatureCloud += *_currentKf->featureCloud;
             mico::ransacAlignment<PointType_>( duplicateCurrentKfFeatureCloud,
                                                _previousKf->featureCloud,
                                                matches,
                                                _transformation,
                                                inliers,
                                                _mRansacMaxDistance,
                                                _mRansacIterations,
                                                _mRansacRefineIterations);
        }else {
            mico::ransacAlignment<PointType_>(  _currentKf->featureCloud,
                                                _previousKf->featureCloud,
                                                matches,
                                                _transformation,
                                                inliers,
                                                _mRansacMaxDistance,
                                                _mRansacIterations,
                                                _mRansacRefineIterations);
        }
        // cv::Mat display = _currentKf->left.clone();
        // for(auto &match:matches){
        //     cv::Point p2 = _currentKf->featureProjections[match.queryIdx];
        //     cv::circle(display, p2, 3, cv::Scalar(0,255,0), 1);
        // }
        // std::cout << matches.size() << std::endl;
        // cv::imshow("FeatureMatcherRansac", display);
        // cv::waitKey();

        logDealer.status("TRANSFORM_BETWEEN_FEATURES", "Inliers between df " + std::to_string(_previousKf->id) + " and kf " + 
                                                        std::to_string(_currentKf->id) + " = " + std::to_string(inliers.size()));
        if (inliers.size() >= _mRansacMinInliers) {
            _currentKf->multimatchesInliersKfs[_previousKf->id];
            _previousKf->multimatchesInliersKfs[_currentKf->id];
            int j = 0;
            for(unsigned int i = 0; i < inliers.size(); i++){
                while(matches[j].queryIdx != inliers[i]){
                    j++;
                }
                _currentKf->multimatchesInliersKfs[_previousKf->id].push_back(matches[j]);
                _previousKf->multimatchesInliersKfs[_currentKf->id].push_back(cv::DMatch(matches[j].trainIdx, matches[j].queryIdx, matches[j].distance));

            }
            return true;
        }else{
            logDealer.error("TRANSFORM_BETWEEN_FEATURES", "Rejecting frame: Num Inliers <" + std::to_string(_mRansacMinInliers));
            return false;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    bool  transformationBetweenFeatures(std::shared_ptr<ClusterFrames<PointType_>> &_previousCf,
                                        std::shared_ptr<DataFrame<PointType_>> &_currentKf,
                                        Eigen::Matrix4f &_transformation,
                                        double _mk_nearest_neighbors,
                                        double _mRansacMaxDistance,
                                        int _mRansacIterations,
                                        double _mRansacMinInliers,
                                        double _mFactorDescriptorDistance,
                                        unsigned _mRansacRefineIterations){

        LoggableInterface<DebugLevel_, OutInterface_> logDealer;
        
        if(_currentKf->multimatchesInliersKfs.find(_previousCf->id) !=  _currentKf->multimatchesInliersKfs.end()){
            // Match already computed
            logDealer.status("TRANSFORM_BETWEEN_FEATURES",  "Match already computed between frame: " + 
                                                            std::to_string(_currentKf->id) + " and cluster " + 
                                                            std::to_string(_previousCf->id));
            return true;
        }
        std::vector<cv::DMatch> matches;
        matchDescriptors(   _currentKf->featureDescriptors,
                            _previousCf->featureDescriptors,
                            matches,
                            _mk_nearest_neighbors,
                            _mFactorDescriptorDistance);
        
        std::vector<int> inliers;
        if(_mk_nearest_neighbors>1){
            typename pcl::PointCloud<PointType_>::Ptr duplicateCurrentKfFeatureCloud = _currentKf->featureCloud;
            *duplicateCurrentKfFeatureCloud += *_currentKf->featureCloud;
             mico::ransacAlignment<PointType_>( duplicateCurrentKfFeatureCloud,
                                                _previousCf->featureCloud,
                                                matches,
                                                _transformation,
                                                inliers,
                                                _mRansacMaxDistance,
                                                _mRansacIterations,
                                                _mRansacRefineIterations);
        }else {
            mico::ransacAlignment<PointType_>(  _currentKf->featureCloud,
                                                _previousCf->featureCloud,
                                                matches,
                                                _transformation,
                                                inliers,
                                                _mRansacMaxDistance,
                                                _mRansacIterations,
                                                _mRansacRefineIterations);
        }
        // cv::Mat display = _currentKf->left.clone();
        // for(auto &match:matches){
        //     cv::Point p2 = _currentKf->featureProjections[match.queryIdx];
        //     cv::circle(display, p2, 3, cv::Scalar(0,255,0), 1);
        // }

        // cv::imshow("FeatureMatcherRansac", display);

        logDealer.status("TRANSFORM_BETWEEN_FEATURES", "Inliers between cf " + std::to_string(_previousCf->id) + " and kf " + 
                                                        std::to_string(_currentKf->id) + " = " + std::to_string(inliers.size()));
        if (inliers.size() >= _mRansacMinInliers) {
            _currentKf->multimatchesInliersCfs[_previousCf->id];
            _previousCf->multimatchesInliersKfs[_currentKf->id];
            int j = 0;
            for(unsigned int i = 0; i < inliers.size(); i++){
                while(matches[j].queryIdx != inliers[i]){
                    j++;
                }
                _currentKf->multimatchesInliersCfs[_previousCf->id].push_back(matches[j]);
                _previousCf->multimatchesInliersKfs[_currentKf->id].push_back(cv::DMatch(matches[j].trainIdx, matches[j].queryIdx, matches[j].distance));

            }
            return true;
        }else{
            logDealer.error("TRANSFORM_BETWEEN_FEATURES", "Rejecting frame: Num Inliers <" + std::to_string(_mRansacMinInliers));
            return false;
        }
    }


    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_/*, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null*/>
    bool  transformationBetweenClusterframes(std::shared_ptr<ClusterFrames<PointType_>> &_trainCf,
                                             std::shared_ptr<ClusterFrames<PointType_>> &_queryCf,
                                             Eigen::Matrix4f &_transformation,
                                             double _mk_nearest_neighbors,
                                             double _mRansacMaxDistance,
                                             int _mRansacIterations,
                                             double _mRansacMinInliers,
                                             double _mFactorDescriptorDistance,
                                             unsigned _mRansacRefineIterations){

        //LoggableInterface<DebugLevel_, OutInterface_> logDealer;
        
        if(_queryCf->multimatchesInliersCfs.find(_trainCf->id) !=  _queryCf->multimatchesInliersCfs.end()){
            // Match already computed
            /*logDealer.status("TRANSFORM_BETWEEN_CLUSTERFRAMES",  "Match already computed between cluster: " + 
                                                                  std::to_string(_queryCf->id) + " and cluster " + 
                                                                  std::to_string(_trainCf->id));*/
            return true;
        }
        std::vector<cv::DMatch> matches;
        matchDescriptors(   _queryCf->featureDescriptors,
                            _trainCf->featureDescriptors,
                            matches,
                            _mk_nearest_neighbors,
                            _mFactorDescriptorDistance);
        
        std::vector<int> inliers;
        if(_mk_nearest_neighbors>1){
            typename pcl::PointCloud<PointType_>::Ptr duplicateQueryCfFeatureCloud = _queryCf->featureCloud;
            *duplicateQueryCfFeatureCloud += *_queryCf->featureCloud;
             mico::ransacAlignment<PointType_>( duplicateQueryCfFeatureCloud,
                                                _trainCf->featureCloud,
                                                matches,
                                                _transformation,
                                                inliers,
                                                _mRansacMaxDistance,
                                                _mRansacIterations,
                                                _mRansacRefineIterations);
        }else {
            mico::ransacAlignment<PointType_>(  _queryCf->featureCloud,
                                                _trainCf->featureCloud,
                                                matches,
                                                _transformation,
                                                inliers,
                                                _mRansacMaxDistance,
                                                _mRansacIterations,
                                                _mRansacRefineIterations);
        }
        

        /*logDealer.status("TRANSFORM_BETWEEN_CLUSTERFRAMES", "Inliers between cf " + std::to_string(_trainCf->id) + " and cf " + 
                                                             std::to_string(_queryCf->id) + " = " + std::to_string(inliers.size()));*/
        if (inliers.size() >= _mRansacMinInliers-5) {   //TODO: Inliers between clusterframes
            _queryCf->multimatchesInliersCfs[_trainCf->id];
            _trainCf->multimatchesInliersCfs[_queryCf->id];
            int j = 0;
            for(unsigned int i = 0; i < inliers.size(); i++){
                while(matches[j].queryIdx != inliers[i]){
                    j++;
                }
                _queryCf->multimatchesInliersCfs[_trainCf->id].push_back(matches[j]);
                _trainCf->multimatchesInliersCfs[_queryCf->id].push_back(cv::DMatch(matches[j].trainIdx, matches[j].queryIdx, matches[j].distance));

            }
            return true;
        }else{
            //logDealer.error("TRANSFORM_BETWEEN_CLUSTERFRAMES", "Rejecting frame: Num Inliers <" + std::to_string(_mRansacMinInliers));
            return false;
        }
    }
}