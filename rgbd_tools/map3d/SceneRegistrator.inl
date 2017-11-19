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
#include <pcl/registration/gicp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>

#include <iostream>
#include <opencv2/core/eigen.hpp>

#include "BundleAdjuster.h"

#include <map3d/utils3d.h>

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline SceneRegistrator<PointType_>::SceneRegistrator(){
        cv::namedWindow("Similarity Matrix", cv::WINDOW_FREERATIO);
        cv::namedWindow("Similarity Matrix red", cv::WINDOW_FREERATIO);
        cv::namedWindow("H Matrix", cv::WINDOW_FREERATIO);
        cv::namedWindow("loopTrace Matrix", cv::WINDOW_FREERATIO);
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
                    //std::cout << transformation << std::endl;
                    if(!refineTransformation( mLastKeyframe, _kf, transformation)){
                        return false;   // reject keyframe.
                    }
                    //std::cout << transformation << std::endl;
                }
                auto t2 = std::chrono::high_resolution_clock::now();

                std::cout <<	"\trough: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() <<
                                ", refine: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "-------------" <<std::endl;

            }

            auto t0 = std::chrono::high_resolution_clock::now();
            Eigen::Affine3f prevPose(mLastKeyframe->pose);
            Eigen::Affine3f lastTransformation(transformation);
            // Compute current position.
            Eigen::Affine3f currentPose = prevPose*lastTransformation;

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
            pcl::transformPointCloudWithNormals(*_kf->cloud, cloud, _kf->pose);

            mSafeMapCopy.lock();
            mMap += cloud;
            mSafeMapCopy.unlock();

            auto t2 = std::chrono::high_resolution_clock::now();

            pcl::VoxelGrid<PointType_> sor;
            sor.setInputCloud (mMap.makeShared());
            sor.setLeafSize (0.01f, 0.01f, 0.01f);
            mSafeMapCopy.lock();
            sor.filter (mMap);
            mSafeMapCopy.unlock();

            auto t3 = std::chrono::high_resolution_clock::now();
            std::cout <<	"\tprepare T: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() <<
                            ", update map: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() <<
                            ", filter map: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << "-------------" <<std::endl;
        }else{
            pcl::PointCloud<PointType_> cloud;
            pcl::transformPointCloudWithNormals(*_kf->cloud, cloud, _kf->pose);

            mSafeMapCopy.lock();
            mMap += cloud;
            mSafeMapCopy.unlock();

            auto t2 = std::chrono::high_resolution_clock::now();

            pcl::VoxelGrid<PointType_> sor;
            sor.setInputCloud (mMap.makeShared());
            sor.setLeafSize (0.01f, 0.01f, 0.01f);
            mSafeMapCopy.lock();
            sor.filter (mMap);
            mSafeMapCopy.unlock();
        }

        // Add keyframe to list.
        mDatabase.addKeyframe(_kf);
        if(mDatabase.numKeyframes() > 1)
            mDatabase.connectKeyframes(_kf->id, mLastKeyframe->id);

        //Update similarity matrix and check for loop closures
        updateSimilarityMatrix(_kf);
        checkLoopClosures();

        // Set kf as last kf
        mLastKeyframe = _kf;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline std::vector<std::shared_ptr<Keyframe<PointType_>>>  SceneRegistrator<PointType_>::keyframes(){
        return mDatabase.keyframes();
    }


    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    pcl::PointCloud<PointType_> SceneRegistrator<PointType_>::map(){
        std::lock_guard<std::mutex> lock(mSafeMapCopy);
        return mMap;
    }

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    std::shared_ptr<Keyframe<PointType_>> SceneRegistrator<PointType_>::lastFrame() const{
        return mLastKeyframe;
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
    inline unsigned SceneRegistrator<PointType_>::baDistanceSearch() const{
        return mDistanceSearch;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline unsigned SceneRegistrator<PointType_>::baSequenceSize() const{
        return mBaSequenceSize;
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
    inline void SceneRegistrator<PointType_>::baDistanceSearch(unsigned _distance) {
        mDistanceSearch = _distance;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline void SceneRegistrator<PointType_>::baSequenceSize(unsigned _sequenceSize){
        mBaSequenceSize = _sequenceSize;
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
    bool SceneRegistrator<PointType_>::initVocabulary(std::string _path){
        mVocabulary.load(_path);
        mDoLoopClosure != mVocabulary.empty();
        return mDoLoopClosure;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::matchDescriptors(const cv::Mat &_des1, const cv::Mat &_des2, std::vector<cv::DMatch> &_inliers) {
        std::vector<cv::DMatch> matches12, matches21;
        cv::BFMatcher featureMatcher;
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
		return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool  SceneRegistrator<PointType_>::transformationBetweenFeatures(std::shared_ptr<Keyframe<PointType_>> &_previousKf, std::shared_ptr<Keyframe<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation){
        if(_currentKf->multimatchesInliersKfs.find(_previousKf->id) !=  _currentKf->multimatchesInliersKfs.end()){
            // Match already computed
            std::cout << "Match alread computed between frames: " <<_currentKf->id << " and " << _previousKf->id << std::endl;
            return true;
        }
        std::vector<cv::DMatch> matches;
        matchDescriptors(_currentKf->featureDescriptors, _previousKf->featureDescriptors, matches);

        std::vector<int> inliers;
        rgbd::ransacAlignment<PointType_>(_currentKf->featureCloud, _previousKf->featureCloud, matches,_transformation, inliers, mRansacMaxDistance, mRansacIterations);

        if (inliers.size() >= 12) {
            _currentKf->multimatchesInliersKfs[_previousKf->id];
            _previousKf->multimatchesInliersKfs[_currentKf->id];
            int j = 0;
            for(int i = 0; i < inliers.size(); i++){
                while(matches[j].queryIdx != inliers[i]){
                    j++;
                }
                _currentKf->multimatchesInliersKfs[_previousKf->id].push_back(matches[j]);
                _previousKf->multimatchesInliersKfs[_currentKf->id].push_back(cv::DMatch(matches[j].trainIdx, matches[j].queryIdx, matches[j].distance));

            }
            return true;
        }else{

            return false;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool SceneRegistrator<PointType_>::refineTransformation(std::shared_ptr<Keyframe<PointType_>> &_previousKf, std::shared_ptr<Keyframe<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation){
        return icpAlignment<PointType_>(_currentKf->cloud,
                                        _previousKf->cloud,
                                        _transformation,
                                        mIcpMaxIterations,
                                        mIcpMaxCorrespondenceDistance,
                                        0.707,
                                        0.3,
                                        0.001,
                                        0.005,
                                        mIcpMaxFitnessScore);
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

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void SceneRegistrator<PointType_>::updateSimilarityMatrix(std::shared_ptr<Keyframe<PointType_>> &_kf) {
        // Computing signature  666 not sure if should be here or in a common place!
        std::vector<cv::Mat> descriptors;
        for(unsigned  r = 0; r < _kf->featureDescriptors.rows; r++){
            descriptors.push_back(_kf->featureDescriptors.row(r));
        }
        mVocabulary.transform(descriptors, _kf->signature);

        // Start Smith-Waterman algorithm
        cv::Mat newMatrix(mDatabase.numKeyframes(), mDatabase.numKeyframes(),CV_32F);   // 666 look for a more efficient way of dealing with the similarity matrix. Now it is recomputed all the time!
        mSimilarityMatrix.copyTo(newMatrix(cv::Rect(0,0,mSimilarityMatrix.cols,mSimilarityMatrix.rows)));
        mSimilarityMatrix = newMatrix;

        // BUILD M matrix, i.e., similarity matrix.
        for(unsigned kfId = 0; kfId < mDatabase.numKeyframes(); kfId++){
            double score = mVocabulary.score(_kf->signature, mDatabase.keyframe(kfId)->signature);
            mSimilarityMatrix.at<float>(kfId, _kf->id) = score;
            mSimilarityMatrix.at<float>(_kf->id, kfId) = score;
        }
        cv::Mat similarityDisplay;
        cv::normalize(mSimilarityMatrix, similarityDisplay, 0.0,1.0,CV_MINMAX);
        cv::imshow("Similarity Matrix", similarityDisplay);
        cv::waitKey(3);

        //cv::Mat Mp = Mr.clone();
        cv::Mat Mp = mSimilarityMatrix.clone();

        if(mDatabase.numKeyframes() > mDistanceSearch){
            // Build H matrix, cummulative matrix
            float penFactor = 0.05;
            mCumulativeMatrix = cv::Mat::zeros(mDatabase.numKeyframes(), mDatabase.numKeyframes(), CV_32F);
            for(unsigned j = mCumulativeMatrix.cols-1 - mDistanceSearch; j >= 1 ; j--){
                for(unsigned i = mCumulativeMatrix.rows-1; i >= j + mDistanceSearch; i--){
                    float diagScore = mCumulativeMatrix.at<float>(i-1, j-1) + Mp.at<float>(i,j);
                    float upScore   = mCumulativeMatrix.at<float>(i-1, j) + Mp.at<float>(i,j) - penFactor;
                    float leftScore = mCumulativeMatrix.at<float>(i, j-1) + Mp.at<float>(i,j) - penFactor;

                    mCumulativeMatrix.at<float>(i,j) = std::max(std::max(diagScore, upScore), leftScore);
                }
            }
            cv::Mat Hdisplay;
            cv::normalize(mCumulativeMatrix, Hdisplay, 0.0,1.0,CV_MINMAX);
            cv::imshow("H Matrix", Hdisplay);
            cv::waitKey(3);
        }

    }

    //-----------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    void SceneRegistrator<PointType_>::checkLoopClosures() {
        if(mDatabase.numKeyframes() > mDistanceSearch){
            // Cover H matrix looking for loop.
            double min, max;
            cv::Point minLoc, maxLoc;
            cv::Mat lastRow = mCumulativeMatrix.row(mDatabase.numKeyframes()-1);
            cv::minMaxLoc(lastRow, &min, &max, &minLoc, &maxLoc);
            cv::Point currLoc = maxLoc;currLoc.y = mDatabase.numKeyframes()-1;
            cv::Mat loopsTraceBack = cv::Mat::zeros(mDatabase.numKeyframes(), mDatabase.numKeyframes(), CV_32F);
            std::vector<std::pair<int, int>> matches;
            while(currLoc.x > 0/*mCumulativeMatrix.rows*/ && currLoc.y>0/* < mCumulativeMatrix.cols*/){
                loopsTraceBack.at<float>(currLoc.y, currLoc.x) = 1;
                matches.push_back({currLoc.y, currLoc.x});
                float diagScore     = mCumulativeMatrix.at<float>(currLoc.y+1, currLoc.x+1);
                float downScore     = mCumulativeMatrix.at<float>(currLoc.y, currLoc.x+1);
                float rightScore    = mCumulativeMatrix.at<float>(currLoc.y+1, currLoc.x);

                if(diagScore> downScore && diagScore > rightScore){
                    currLoc.x--;//++;
                    currLoc.y--;//++;
                }else if(downScore > diagScore && downScore > rightScore){
                    currLoc.y--;//++;
                }else {
                    currLoc.x--;//++;
                }

                if(mCumulativeMatrix.at<float>(currLoc.y, currLoc.x) <= 0.01){
                    break;
                }
            }
            cv::Mat loopsTraceBackDisplay;
            cv::normalize(loopsTraceBack, loopsTraceBackDisplay, 0.0,1.0,CV_MINMAX);
            cv::imshow("loopTrace Matrix", loopsTraceBackDisplay);
            cv::waitKey(3);
            if(matches.size() > mBaSequenceSize){ // Loop closure detected! update kfs and world dictionary and perform the optimization.
                for(unsigned i = 0; i < matches.size(); i++){
                    Eigen::Matrix4f transformation; // Not used at all but needded in the interface
                    auto kf1 = mDatabase.keyframe(matches[i].first);
                    auto kf2 = mDatabase.keyframe(matches[i].second);
                    transformationBetweenFeatures(kf1,kf2, transformation);
                    // Update worldDictionary.
                    mDatabase.connectKeyframes(kf2->id, kf1->id);
                }

                std::cout << "performing bundle adjustment!" << std::endl;

                if(!mAlreadyBaThread){
                    mAlreadyBaThread = true;
                    if(mBaThread.joinable())
                        mBaThread.join();

                    mKeyframesBa = mDatabase.keyframes();
                    mBaThread = std::thread([&](){
                        // Perform loop closure
                        rgbd::BundleAdjuster<PointType_> ba;
                        ba.keyframes(mKeyframesBa);
                        ba.optimize();
                        //  mKeyframes =ba.keyframes();

                        pcl::PointCloud<PointType_> map;
                        for(auto &kf:mKeyframesBa){
                            pcl::PointCloud<PointType_> cloud;
                            pcl::transformPointCloudWithNormals(*kf->cloud, cloud, kf->pose);
                            map += cloud;
                        }
                        pcl::VoxelGrid<PointType_> sor;
                        sor.setInputCloud (mMap.makeShared());
                        sor.setLeafSize (0.01f, 0.01f, 0.01f);
                        sor.filter (map);
                        mSafeMapCopy.lock();
                        mMap = map;
                        mSafeMapCopy.unlock();
                        mAlreadyBaThread = false;
                    });
                }
            }
        }
    }

}

