////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include <thread>
#include <chrono>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <iostream>

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline SceneRegistrator<PointType_>::SceneRegistrator(){
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::addKeyframe(std::shared_ptr<Keyframe<PointType_>> &_kf){
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

        if(mLastKeyframe != nullptr){
            if((_kf->featureCloud == nullptr || _kf->featureCloud->size() ==0)) {
                auto t1 = std::chrono::high_resolution_clock::now();
                // Fine rotation.
                if(!refineTransformation( mLastKeyframe, _kf, transformation)){
                    return false;   // reject keyframe.
                }
                auto t2 = std::chrono::high_resolution_clock::now();

                std::cout <<"Refine: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "-------------" <<std::endl;
            }else{
                // Match feature points.
                auto t0 = std::chrono::high_resolution_clock::now();
                // Compute initial rotation.
                if(!transformationBetweenFeatures( mLastKeyframe, _kf, transformation)){
                    return false;   // reject keyframe.
                }

                auto t1 = std::chrono::high_resolution_clock::now();

                if(mIcpEnabled){
                    // Fine rotation.
                    if(!refineTransformation( mLastKeyframe, _kf, transformation)){
                        return false;   // reject keyframe.
                    }
                }
                auto t2 = std::chrono::high_resolution_clock::now();

                std::cout <<	"\trough: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() <<
                                ", refine: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "-------------" <<std::endl;

            }

            auto t0 = std::chrono::high_resolution_clock::now();
            Eigen::Affine3f prevPose = Eigen::Translation3f(mLastKeyframe->position)*mLastKeyframe->orientation;
            Eigen::Affine3f lastTransformation(transformation);
            // Compute current position.
            Eigen::Affine3f currentPose = lastTransformation*prevPose;

            // Check transformation
            Eigen::Vector3f ea = transformation.block<3,3>(0,0).eulerAngles(0, 1, 2);
            float angleThreshold = 20.0;///180.0*M_PI;
            float distanceThreshold = 0.3;
            if((abs(ea[0]) + abs(ea[1]) + abs(ea[2])) > angleThreshold || transformation.block<3,1>(0,3).norm() > distanceThreshold){
                std::cout << "Large transformation! not accepted KF" << std::endl;
                return false;
            }

            _kf->position = currentPose.translation();
            _kf->orientation = currentPose.rotation();
            _kf->pose = currentPose.matrix();
            auto t1 = std::chrono::high_resolution_clock::now();

            pcl::PointCloud<PointType_> cloud;
            pcl::transformPointCloudWithNormals(*_kf->cloud, cloud, currentPose);
            mMap += cloud;

            auto t2 = std::chrono::high_resolution_clock::now();

            pcl::VoxelGrid<PointType_> sor;
            sor.setInputCloud (mMap.makeShared());
            sor.setLeafSize (0.02f, 0.02f, 0.02f);
            sor.filter (mMap);

            auto t3 = std::chrono::high_resolution_clock::now();
            std::cout <<	"\tprepare T: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() <<
                            ", update map: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() <<
                            ", filter map: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << "-------------" <<std::endl;
            fillDictionary(_kf);
        }else{
//            // init dictionary with first cloud
//            for(unsigned idx = 0; idx < _kf->featureCloud->size(); idx++){
//                mWorldDictionary[idx] = std::shared_ptr<Word>(new Word);
//                mWorldDictionary[idx]->id       = idx;
//                mWorldDictionary[idx]->point    = {_kf->featureCloud->at(idx).x, _kf->featureCloud->at(idx).y, _kf->featureCloud->at(idx).z};
//                mWorldDictionary[idx]->frames   = {_kf->id};

//                _kf->wordsReference.push_back(mWorldDictionary[idx]);
//            }
        }

        // Add keyframe to list.
        mKeyframesQueue.push_back(_kf);
        mLastKeyframe = _kf;
//rgbd::Gui::get()->pause();
        const int cBaQueueSize = 5;
        if(mKeyframesQueue.size() == cBaQueueSize){
            mBA.keyframes(mKeyframesQueue);
            mBA.optimize();
            mKeyframes.insert(mKeyframes.end(), mKeyframesQueue.begin(), mKeyframesQueue.end());
            mKeyframesQueue.clear();

            for(auto &kf:mKeyframes){
                mMap.clear();
                pcl::PointCloud<PointType_> cloud;
                pcl::transformPointCloudWithNormals(*_kf->cloud, cloud, kf->pose);
                mMap += cloud;
            }

            rgbd::Gui::get()->pause();
        }

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline std::vector<std::shared_ptr<Keyframe<PointType_>>>  SceneRegistrator<PointType_>::keyframes() const{
        return mKeyframes;
    }


    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    pcl::PointCloud<PointType_> SceneRegistrator<PointType_>::map() const{
        return mMap;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::baMinError       () const{
        return mBA.minError();
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline unsigned SceneRegistrator<PointType_>::baIterations     () const{
        return mBA.iterations();
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline unsigned SceneRegistrator<PointType_>::baMinAparitions  () const{
        return mBA.minAparitions();
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::descriptorDistanceFactor     () const{
        return mFactorDescriptorDistance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline int SceneRegistrator<PointType_>::ransacIterations             () const{
        return mRansacIterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::ransacMaxDistance   () const{
        return mRansacMaxDistance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline int SceneRegistrator<PointType_>::ransacMinInliers   () const{
        return mRansacMinInliers;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::icpMaxTransformationEpsilon() const{
        return mIcpMaxTransformationEpsilon;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::icpMaxCorrespondenceDistance() const{
        return mIcpMaxCorrespondenceDistance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::icpVoxelDistance() const{
        return mIcpVoxelDistance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline double SceneRegistrator<PointType_>::icpMaxFitnessScore() const{
        return  mIcpMaxFitnessScore;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline int SceneRegistrator<PointType_>::icpMaxIterations() const{
        return mIcpMaxIterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::baMinError         (double _error){
        mBA.minError(_error);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::baIterations       (unsigned _iterations){
        mBA.iterations(_iterations);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::baMinAparitions    (unsigned _aparitions){
        mBA.minAparitions(_aparitions);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::descriptorDistanceFactor       (double _factor){
        mFactorDescriptorDistance = _factor;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::ransacIterations               (int _iterations){
        mRansacIterations = _iterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::ransacMaxDistance     (double _maxDistance){
        mRansacMaxDistance = _maxDistance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::ransacMinInliers     (int _minInliers){
        mRansacMinInliers = _minInliers;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::icpMaxTransformationEpsilon        (double _maxEpsilon){
        mIcpMaxTransformationEpsilon = _maxEpsilon;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::icpMaxCorrespondenceDistance       (double _distance){
        mIcpMaxCorrespondenceDistance = _distance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::icpVoxelDistance     (double _distance){
        mIcpVoxelDistance = _distance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::icpMaxFitnessScore (double _maxScore){
        mIcpMaxFitnessScore = _maxScore;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::icpMaxIterations (int _maxIters){
        mIcpMaxIterations = _maxIters;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::matchDescriptors(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers) {
        std::vector<cv::DMatch> matches12, matches21;
        cv::FlannBasedMatcher featureMatcher;
        featureMatcher.match(_des1, _des2, matches12);
        featureMatcher.match(_des2, _des1, matches21);

        double max_dist = 0; double min_dist = 999999;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < _des1.rows; i++ ) {
            double dist = matches12[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        // symmetry test.
        for(std::vector<cv::DMatch>::iterator it12 = matches12.begin(); it12 != matches12.end(); it12++){
            for(std::vector<cv::DMatch>::iterator it21 = matches21.begin(); it21 != matches21.end(); it21++){
                if(it12->queryIdx == it21->trainIdx && it21->queryIdx == it12->trainIdx){
                    if(it12->distance <= min_dist*mFactorDescriptorDistance){
                        _inliers.push_back(*it12);
                    }
                    break;
                }
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool  SceneRegistrator<PointType_>::transformationBetweenFeatures(std::shared_ptr<Keyframe<PointType_>> &_previousKf, std::shared_ptr<Keyframe<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation){
        matchDescriptors(_currentKf->featureDescriptors, _previousKf->featureDescriptors, _currentKf->matchesPrev);

        std::vector<int> source_indices (_currentKf->matchesPrev.size());
        std::vector<int> target_indices (_currentKf->matchesPrev.size());

        // Copy the query-match indices
        for (int i = 0; i < (int)_currentKf->matchesPrev.size(); ++i) {
            source_indices[i] = _currentKf->matchesPrev[i].queryIdx;
            target_indices[i] = _currentKf->matchesPrev[i].trainIdx;
        }

        typename pcl::SampleConsensusModelRegistration<PointType_>::Ptr model(new pcl::SampleConsensusModelRegistration<PointType_>(_currentKf->featureCloud, source_indices));

        // Pass the target_indices
        model->setInputTarget (_previousKf->featureCloud, target_indices);

        // Create a RANSAC model
        pcl::RandomSampleConsensus<PointType_> sac (model, mRansacMaxDistance);

        sac.setMaxIterations(mRansacIterations);
        // Compute the set of inliers
        if(sac.computeModel()) {
            std::vector<int> inliers;
            Eigen::VectorXf model_coefficients;

            sac.getInliers(inliers);

            sac.getModelCoefficients (model_coefficients);
/////////////////////////////
            int refineIterations=5;
            if (refineIterations > 0) {
                double error_threshold = mRansacMaxDistance;
                int refine_iterations = 0;
                bool inlier_changed = false, oscillating = false;
                std::vector<int> new_inliers, prev_inliers = inliers;
                std::vector<size_t> inliers_sizes;
                Eigen::VectorXf new_model_coefficients = model_coefficients;
                do {
                    // Optimize the model coefficients
                    model->optimizeModelCoefficients (prev_inliers, new_model_coefficients, new_model_coefficients);
                    inliers_sizes.push_back (prev_inliers.size ());

                    // Select the new inliers based on the optimized coefficients and new threshold
                    model->selectWithinDistance (new_model_coefficients, error_threshold, new_inliers);
                    //UDEBUG("RANSAC refineModel: Number of inliers found (before/after): %d/%d, with an error threshold of %f.",
                    //        (int)prev_inliers.size (), (int)new_inliers.size (), error_threshold);

                    if (new_inliers.empty ()) {
                        ++refine_iterations;
                        if (refine_iterations >= refineIterations) {
                            break;
                        }
                        continue;
                    }

                    // Estimate the variance and the new threshold
                    double variance = model->computeVariance ();
                    double refineSigma = 3.0;
                    error_threshold = std::min (mRansacMaxDistance, refineSigma * sqrt(variance));

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
                }
                while (inlier_changed && ++refine_iterations < refineIterations);

                // If the new set of inliers is empty, we didn't do a good job refining
                if (new_inliers.empty ()){
                    //UWARN ("RANSAC refineModel: Refinement failed: got an empty set of inliers!");
                }

                if (oscillating){
                    //UDEBUG("RANSAC refineModel: Detected oscillations in the model refinement.");
                }

                std::swap (inliers, new_inliers);
                model_coefficients = new_model_coefficients;
            }
/////////////////////////////
            if (inliers.size() >= 3) {
                int j = 0;
                for(int i = 0; i < inliers.size(); i++){
                    while(_currentKf->matchesPrev[j].queryIdx != inliers[i]){
                        j++;
                    }
                    _currentKf->ransacInliers.push_back(_currentKf->matchesPrev[j]);
                }

                double covariance = model->computeVariance();


                // get best transformation
                Eigen::Matrix4f bestTransformation;
                bestTransformation.row (0) = model_coefficients.segment<4>(0);
                bestTransformation.row (1) = model_coefficients.segment<4>(4);
                bestTransformation.row (2) = model_coefficients.segment<4>(8);
                bestTransformation.row (3) = model_coefficients.segment<4>(12);
                _transformation = bestTransformation;

                return true;
            }
        }

        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::refineTransformation(std::shared_ptr<Keyframe<PointType_>> &_previousKf, std::shared_ptr<Keyframe<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation){
        // Align
        pcl::IterativeClosestPointNonLinear<PointType_, PointType_> pcJoiner;
        pcJoiner.setTransformationEpsilon (mIcpMaxTransformationEpsilon);
        pcJoiner.setMaxCorrespondenceDistance (mIcpMaxCorrespondenceDistance);
        pcJoiner.setMaximumIterations(3);
		//pcJoiner.setUseReciprocalCorrespondences(true);

		pcl::registration::CorrespondenceRejectorOneToOne::Ptr corr_rej_one_to_one(new pcl::registration::CorrespondenceRejectorOneToOne);
		pcJoiner.addCorrespondenceRejector(corr_rej_one_to_one);

        pcl::PointCloud<PointType_> srcCloud;
        pcl::PointCloud<PointType_> tgtCloud;

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*_previousKf->cloud, tgtCloud, indices);
        pcl::removeNaNFromPointCloud(*_currentKf->cloud, srcCloud, indices);

        pcl::VoxelGrid<PointType_> sor;
        sor.setInputCloud (srcCloud.makeShared());
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter (srcCloud);
        sor.setInputCloud (tgtCloud.makeShared());
        sor.filter (tgtCloud);

        pcl::StatisticalOutlierRemoval<PointType_> sor2;
        sor2.setMeanK (50);
        sor2.setStddevMulThresh (1.0);
        sor2.setInputCloud (srcCloud.makeShared());
        sor2.filter (srcCloud);
        sor2.setInputCloud (tgtCloud.makeShared());
        sor2.filter (tgtCloud);

        pcJoiner.setInputTarget(tgtCloud.makeShared());
        pcl::PointCloud<PointType_> alignedCloud;
        Eigen::Matrix4f prevTransformation = _transformation;
        bool hasConvergedInSomeIteration = false;
        int i = 0;
        std::cout << "ICP iterations: ";
        for (i = 0; i < mIcpMaxIterations; ++i){
            // Estimate
            std::cout << i << ", ";
            pcJoiner.setInputSource(srcCloud.makeShared());
            pcJoiner.align(alignedCloud, _transformation);
            //accumulate transformation between each Iteration
            _transformation = pcJoiner.getFinalTransformation();
            if (pcJoiner.getFinalTransformation().hasNaN()) {
                std::cout << "--> MAP: Intermedial iteration of ICP throw transformation with NaN, skiping it and continuing iterations" << std::endl;
                break;
            }
            //if the difference between this transformation and the previous one
            //is smaller than the threshold, refine the process by reducing
            //the maximal correspondence distance
            if (fabs((pcJoiner.getLastIncrementalTransformation() - prevTransformation).sum()) < pcJoiner.getTransformationEpsilon()) {
                pcJoiner.setMaxCorrespondenceDistance(pcJoiner.getMaxCorrespondenceDistance()*0.9);
            }

            prevTransformation = pcJoiner.getLastIncrementalTransformation();
            hasConvergedInSomeIteration |= pcJoiner.hasConverged();

            if (pcJoiner.getMaxCorrespondenceDistance() < 0.001) {
                break;
            }
        }

        cout << "--> MAP: Exiting ICP iterations in " << i+1 << "/" << mIcpMaxIterations << endl;
        if (_transformation.hasNaN()) {
            cerr << "--> MAP:  ---> CRITICAL ERROR! Transformation has nans!!! <---" << endl;
            return false;
        }

        return (pcJoiner.hasConverged() || hasConvergedInSomeIteration) && pcJoiner.getFitnessScore() < mIcpMaxFitnessScore;
    }


    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::icpEnabled(bool _enable) {
        mIcpEnabled = _enable;
    }


    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::icpEnabled() const {
        return mIcpEnabled;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::fillDictionary(std::shared_ptr<Keyframe<PointType_>> &_kf){
        auto &prevKf = mLastKeyframe;
        pcl::PointCloud<PointType_> transCloud;
        pcl::transformPointCloud(*_kf->featureCloud, transCloud, _kf->pose);    // 666 Transform only chosen points not all.

        for(unsigned inlierIdx = 0; inlierIdx < _kf->ransacInliers.size(); inlierIdx++){
            std::shared_ptr<Word> prevWord = nullptr;
            int inlierIdxInCurrent = _kf->ransacInliers[inlierIdx].queryIdx;
            int inlierIdxInPrev = _kf->ransacInliers[inlierIdx].trainIdx;
            for(auto &w: prevKf->wordsReference){
                if(w->idxInKf[prevKf->id] == inlierIdxInPrev){
                    prevWord = w;
                    break;
                }
            }
            if(prevWord){
                prevWord->frames.push_back(_kf->id);
                prevWord->projections[_kf->id] = {_kf->featureProjections[inlierIdxInCurrent].x, _kf->featureProjections[inlierIdxInCurrent].y};
                prevWord->idxInKf[_kf->id] = inlierIdxInCurrent;
                _kf->wordsReference.push_back(prevWord);
            }else{
                //cv::Mat display;
                //cv::hconcat(prevKf->left, _kf->left, display);
                //cv::Point2i cvp = _kf->featureProjections[inlierIdxInCurrent]; cvp.x += prevKf->left.cols;
                //cv::line(display, prevKf->featureProjections[inlierIdxInPrev], cvp, cv::Scalar(0,255,0));
                //cvp = prevKf->featureProjections[inlierIdxInPrev]; cvp.x += prevKf->left.cols;
                //cv::line(display, _kf->featureProjections[inlierIdxInCurrent], cvp, cv::Scalar(0,0,255));
                //cv::imshow("sadadad", display);
                //cv::waitKey();
                int wordId = mWorldDictionary.size();
                mWorldDictionary[wordId] = std::shared_ptr<Word>(new Word);
                mWorldDictionary[wordId]->id        = wordId;
                mWorldDictionary[wordId]->point     = {transCloud.at(inlierIdxInCurrent).x, transCloud.at(inlierIdxInCurrent).y, transCloud.at(inlierIdxInCurrent).z};
                mWorldDictionary[wordId]->frames    = {prevKf->id, _kf->id};
                mWorldDictionary[wordId]->projections[prevKf->id] = {prevKf->featureProjections[inlierIdxInPrev].x, prevKf->featureProjections[inlierIdxInPrev].y};
                mWorldDictionary[wordId]->projections[_kf->id] = {_kf->featureProjections[inlierIdxInCurrent].x, _kf->featureProjections[inlierIdxInCurrent].y};
                mWorldDictionary[wordId]->idxInKf[prevKf->id] = inlierIdxInPrev;
                mWorldDictionary[wordId]->idxInKf[_kf->id] = inlierIdxInCurrent;
                _kf->wordsReference.push_back(mWorldDictionary[wordId]);
                prevKf->wordsReference.push_back(mWorldDictionary[wordId]);

                //if(wordId < 10){
                //    std::cout << prevKf->id << ", " << wordId <<", "<< mWorldDictionary[wordId]->projections[prevKf->id][0] << ", " << mWorldDictionary[wordId]->projections[prevKf->id][1] <<std::endl;
                //    std::cout << _kf->id << ", "    << wordId <<", "<< mWorldDictionary[wordId]->projections[_kf->id][0] << ", " << mWorldDictionary[wordId]->projections[_kf->id][1] << std::endl;
                //}
            }
        }
    }


}

