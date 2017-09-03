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

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::addKeyframe(Keyframe<PointType_> &_kf){
        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

        if(mKeyframes.size() != 0){
            if((_kf.featureCloud == nullptr || _kf.featureCloud->size() ==0)) {
                auto t1 = std::chrono::high_resolution_clock::now();
                // Fine rotation.
                if(!refineTransformation( mKeyframes.back(), _kf, transformation)){
                    return false;   // reject keyframe.
                }
                auto t2 = std::chrono::high_resolution_clock::now();

                std::cout <<"Refine: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "-------------" <<std::endl;
            }else{
                // Match feature points.
                auto t0 = std::chrono::high_resolution_clock::now();
                // Compute initial rotation.
                if(!transformationBetweenFeatures( mKeyframes.back(), _kf, transformation)){
                    return false;   // reject keyframe.
                }

                auto t1 = std::chrono::high_resolution_clock::now();

                if(mIcpEnabled){
                    // Fine rotation.
                    if(!refineTransformation( mKeyframes.back(), _kf, transformation)){
                        return false;   // reject keyframe.
                    }
                }
                auto t2 = std::chrono::high_resolution_clock::now();

                std::cout <<	"\trough: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() <<
                                ", refine: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "-------------" <<std::endl;

            }

            Eigen::Affine3f prevPose = Eigen::Translation3f(mKeyframes.back().position)*mKeyframes.back().orientation;
            Eigen::Affine3f lastTransformation(transformation);
            // Compute current position.
            Eigen::Affine3f currentPose = lastTransformation*prevPose;

            _kf.position = currentPose.translation();
            _kf.orientation = currentPose.rotation();
            _kf.pose = currentPose.matrix();

            // Check transformation
            Eigen::Vector3f ea = transformation.block<3,3>(0,0).eulerAngles(0, 1, 2);
            if(ea[0] > 20 || ea[1] > 20 || ea[2] > 20 || transformation.block<3,1>(0,3).norm() > 0.3){
                std::cout << "Large transformation! not accepted KF" << std::endl;
                return false;
            }

            if(mUpdateMapVisualization){
                for(auto &kf:mKeyframes){
                    mMap.clear();
                    pcl::PointCloud<PointType_> cloud;
                    Eigen::Matrix4f pose = kf.pose;
                    pcl::transformPointCloudWithNormals(*_kf.cloud, cloud, pose);
                    mMap += cloud;
                }
            }else{
                pcl::PointCloud<PointType_> cloud;
                pcl::transformPointCloudWithNormals(*_kf.cloud, cloud, currentPose);
                mMap += cloud;
            }

            pcl::VoxelGrid<PointType_> sor;
            sor.setInputCloud (mMap.makeShared());
            sor.setLeafSize (0.01f, 0.01f, 0.01f);
            sor.filter (mMap);

            fillDictionary(_kf);
        }else{
            // init dictionary with first cloud
            for(unsigned idx = 0; idx < _kf.featureCloud->size(); idx++){
                mWorldDictionary[idx] = std::shared_ptr<Word>(new Word);
                mWorldDictionary[idx]->id       = idx;
                mWorldDictionary[idx]->point    = {_kf.featureCloud->at(idx).x, _kf.featureCloud->at(idx).y, _kf.featureCloud->at(idx).z};
                mWorldDictionary[idx]->frames   = {_kf.id};

                _kf.wordsReference.push_back(mWorldDictionary[idx]);
            }
        }

        // Add keyframe to list.
        mKeyframes.push_back(_kf);
        mBaCounter++;

        const int cBaQueueSize = 4;
        if(mBaCounter == cBaQueueSize){
            if(mKeyframes.size() >= cBaQueueSize){
                std::vector<Keyframe<PointType_>, Eigen::aligned_allocator <Keyframe<PointType_>>> usedKfs(mKeyframes.end()-cBaQueueSize, mKeyframes.end());
                mBA.keyframes(usedKfs);
                mBA.optimize();
                auto newKfs = mBA.keyframes();
                for(unsigned kfIdx = 0; kfIdx < 0; kfIdx++){
                    mKeyframes[mKeyframes.size() - (cBaQueueSize - kfIdx) - 1] = newKfs[kfIdx];
                }
                mBaCounter = 0;
                mUpdateMapVisualization = true;
            }
        }

        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline std::vector<Keyframe<PointType_>, Eigen::aligned_allocator <Keyframe<PointType_>>>  SceneRegistrator<PointType_>::keyframes() const{
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
    inline bool  SceneRegistrator<PointType_>::transformationBetweenFeatures(Keyframe<PointType_> &_previousKf, Keyframe<PointType_> &_currentKf, Eigen::Matrix4f &_transformation){
        // Get matches
        matchDescriptors(_currentKf.featureDescriptors, _previousKf.featureDescriptors, _currentKf.matchesPrev);

        mRansacAligner.srcKf = _currentKf;
        mRansacAligner.tgtKf = _previousKf;
        mRansacAligner.sourceTarget(*_currentKf.featureCloud, *_previousKf.featureCloud, _currentKf.matchesPrev);
        mRansacAligner.maxIters(mRansacIterations);
        mRansacAligner.maxDistance(mRansacMaxDistance);
        mRansacAligner.minInliers(mRansacMinInliers);
        if(!mRansacAligner.run()){
            std::cout << "Cant align clouds using ransac P2P" << std::endl;
            return false;
        }

        _transformation = mRansacAligner.transformation();

        mRansacAligner.inliers(_currentKf.ransacInliers);

        return true;    //666 TODO check if transformation if valid and so on...
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::refineTransformation(Keyframe<PointType_> &_previousKf, Keyframe<PointType_> &_currentKf, Eigen::Matrix4f &_transformation){
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
        pcl::removeNaNFromPointCloud(*_previousKf.cloud, tgtCloud, indices);
        pcl::removeNaNFromPointCloud(*_currentKf.cloud, srcCloud, indices);

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
    inline void SceneRegistrator<PointType_>::fillDictionary(Keyframe<PointType_> &_kf){
        auto &prevKf = mKeyframes.back();
        // 666 is it possible to optimize inliers?
        for(unsigned idx = 0; idx < _kf.featureCloud->size(); idx++){
            bool isInlier = false;
            int inlierIdx = 0;
            for(inlierIdx = 0; inlierIdx < _kf.ransacInliers.size(); inlierIdx++){
                if(_kf.ransacInliers[inlierIdx].queryIdx == idx){
                    isInlier = true;
                    break;
                }
            }
            if(isInlier){
                int prevId = prevKf.wordsReference[_kf.ransacInliers[inlierIdx].trainIdx]->id;
                mWorldDictionary[prevId]->frames.push_back(_kf.id);
                _kf.wordsReference.push_back(mWorldDictionary[prevId]);
            }else{
                int wordId = mWorldDictionary.rbegin()->second->id + 1;
                mWorldDictionary[wordId] = std::shared_ptr<Word>(new Word);
                mWorldDictionary[wordId]->id        = wordId;
                mWorldDictionary[wordId]->point     = {_kf.featureCloud->at(idx).x, _kf.featureCloud->at(idx).y, _kf.featureCloud->at(idx).z};
                mWorldDictionary[wordId]->frames    = {_kf.id};
                _kf.wordsReference.push_back(mWorldDictionary[wordId]);
            }
        }
    }

}

