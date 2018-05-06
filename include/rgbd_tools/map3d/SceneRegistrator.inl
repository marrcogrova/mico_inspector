//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
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

#include <thread>
#include <chrono>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <rgbd_tools/map3d/utils3d.h>


namespace rgbd{
//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline SceneRegistrator<PointType_>::SceneRegistrator(){

}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline bool SceneRegistrator<PointType_>::addDataframe(std::shared_ptr<DataFrame<PointType_>> &_kf){
    // Localization
    if(locateDataframe(_kf)){
        // Mapping
        // Add keyframe to list.
        if(mDatabase.addDataframe(_kf,mk_nearest_neighbors,mRansacMaxDistance,mRansacIterations,mRansacMinInliers,mFactorDescriptorDistance)){
            mLastCluster = lastCluster();
            //Check for loop closures
            mLoopClosureDetector.update(mDatabase);
        }
    }else {
        return false;
    }
    // Set kf as last kf
    mLastKeyframe = _kf;
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline bool SceneRegistrator<PointType_>::locateDataframe(std::shared_ptr<DataFrame<PointType_>> &_kf){
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    if(mLastKeyframe != nullptr){
        if(_kf->featureCloud == nullptr && _kf->cloud== nullptr && _kf->left.rows != 0){
            std::vector<cv::DMatch> matches;
            matchDescriptors(_kf->featureDescriptors, mLastKeyframe->featureDescriptors, matches,mk_nearest_neighbors,mFactorDescriptorDistance);

            std::vector<cv::Point2f> projsForPoseCurrent(matches.size()), projsForPosePrev(matches.size());
            for(unsigned i = 0; i < matches.size(); i++){
                projsForPoseCurrent[i] = _kf->featureProjections[matches[i].queryIdx];
                projsForPosePrev[i] = mLastKeyframe->featureProjections[matches[i].trainIdx];
            }

            if(matches.size() < 20){
                return false;
            }

            cv::Mat R, t, maskInliers;
            cv::Mat essential = cv::findEssentialMat(projsForPoseCurrent, projsForPosePrev,_kf->intrinsic,cv::RANSAC, 0.999, 1, maskInliers);
            cv::recoverPose(essential, projsForPoseCurrent, projsForPosePrev,_kf->intrinsic,R, t, maskInliers);

            for(unsigned i = 0 ; i < 3 ; i++){
                for(unsigned j = 0; j < 3; j++){
                    transformation(i,j) = R.at<double>(i,j);
                }
                transformation(i,3) = t.at<double>(i);
            }
            int numInliers = cv::sum(maskInliers)[0];
            if (numInliers >= 12) { // PARAMETRIZE INLIERS This process is somehow repeated in transformation between features! might be good to colapse!
                _kf->multimatchesInliersKfs[mLastKeyframe->id];
                mLastKeyframe->multimatchesInliersKfs[_kf->id];
                for(unsigned i = 0; i < maskInliers.rows; i++){
                    if(maskInliers.at<uchar>(i) == 1){
                        _kf->multimatchesInliersKfs[mLastKeyframe->id].push_back(matches[i]);
                        mLastKeyframe->multimatchesInliersKfs[_kf->id].push_back(cv::DMatch(matches[i].trainIdx, matches[i].queryIdx, matches[i].distance));
                    }
                }
            }else {
                return false;
            }
        }else if((_kf->featureCloud == nullptr || _kf->featureCloud->size() ==0)) { // No feature cloud
            auto t1 = std::chrono::high_resolution_clock::now();
            // Fine rotation.
            if(!refineTransformation( mLastKeyframe, _kf, transformation)){
                return false;   // reject keyframe.
            }
            auto t2 = std::chrono::high_resolution_clock::now();

            std::cout <<"Refine: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "-------------" <<std::endl;
        }else { // Feature cloud and dense cloud
            // Compute initial rotation.
            if(mOnlyLocalizationMode){
                if(!transformationBetweenClusterWords<PointType_>( mLastCluster,_kf,transformation,mk_nearest_neighbors,mRansacMaxDistance,mRansacIterations,mRansacMinInliers,mFactorDescriptorDistance)){
                    return false;   // reject keyframe.
                }
            }else{ 
                if(!transformationBetweenFeatures<PointType_>( mLastKeyframe,_kf,transformation,mk_nearest_neighbors,mRansacMaxDistance,mRansacIterations,mRansacMinInliers,mFactorDescriptorDistance)){
                    return false;   // reject keyframe.
                }
            }
            if(mIcpEnabled){
                // Fine rotation.
                if(!refineTransformation( mLastKeyframe, _kf, transformation)){
                    return false;   // reject keyframe.
                }
            }
        }

        auto t0 = std::chrono::high_resolution_clock::now();
        Eigen::Affine3f prevPose(mLastKeyframe->pose);
        Eigen::Affine3f lastTransformation(transformation);
        // Compute current position.
        Eigen::Affine3f currentPose = prevPose*lastTransformation;

        // Check transformation
        Eigen::Vector3f ea = transformation.block<3,3>(0,0).eulerAngles(0, 1, 2);
        float angleThreshold = 20.0;///180.0*M_PI;
        float distanceThreshold = 3.0;
        if((abs(ea[0]) + abs(ea[1]) + abs(ea[2])) > angleThreshold || transformation.block<3,1>(0,3).norm() > distanceThreshold){
            std::cout << "Large transformation! not accepted KF" << std::endl;
            return false;
        }

        _kf->position = currentPose.translation();
        _kf->orientation = currentPose.rotation();
        _kf->pose = currentPose.matrix();
    }

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline std::unordered_map<int, std::shared_ptr<ClusterFrames<PointType_>>>  SceneRegistrator<PointType_>::clusters(){
    return mDatabase.clusters();
}

//-----------------------------------------------------------------------------------------------------------------
template<typename PointType_>
std::shared_ptr<DataFrame<PointType_>> SceneRegistrator<PointType_>::lastFrame() const{
    return mLastKeyframe;
}

//-----------------------------------------------------------------------------------------------------------------
template<typename PointType_>
std::shared_ptr<ClusterFrames<PointType_>> SceneRegistrator<PointType_>::lastCluster() const{
    return mDatabase.rlastCluster();
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline double SceneRegistrator<PointType_>::baMinError       () const{
    return 0;//mBA.minError();
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline unsigned SceneRegistrator<PointType_>::baIterations     () const{
    return 0;//mBA.iterations();
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline unsigned SceneRegistrator<PointType_>::baMinAparitions  () const{
    return 0;//mBA.minAparitions();
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
    //mBA.minError(_error);
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::baIterations       (unsigned _iterations){
    //mBA.iterations(_iterations);
}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::baMinAparitions    (unsigned _aparitions){
    //mBA.minAparitions(_aparitions);
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
    return mDatabase.initVocabulary(_path);
}


//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline bool SceneRegistrator<PointType_>::refineTransformation(std::shared_ptr<DataFrame<PointType_>> &_previousKf, std::shared_ptr<DataFrame<PointType_>> &_currentKf, Eigen::Matrix4f &_transformation){
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
inline void SceneRegistrator<PointType_>::changeClusterScore(int _dbow2Score){
    mDatabase.changeDBow2Score(_dbow2Score);
}


//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline void SceneRegistrator<PointType_>::k_nearest_neighbors(int _k_nearest_neighbors) {
    mk_nearest_neighbors = _k_nearest_neighbors;

}

//---------------------------------------------------------------------------------------------------------------------
template<typename PointType_>
inline int SceneRegistrator<PointType_>::k_nearest_neighbors() const{
    return  mk_nearest_neighbors;
}

}
