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


#include "cvsba/cvsba.h"

#include <unordered_map>

#include <rgbd_tools/utils/Gui.h>

#include <pcl/common/transforms.h>

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline double BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::minError       () const{
        return mBaMinError;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline unsigned BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::iterations     () const{
        return mBaIterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline unsigned BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::minWords     () const{
        return mMinWords;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::minError         (double _error){
        mBaMinError = _error;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::minWords         (unsigned _minWords){
        mMinWords = _minWords;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::iterations       (unsigned _iterations){
        mBaIterations = _iterations;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::minAparitions       (unsigned  _aparitions){
        mBaMinAparitions = _aparitions;
    }

    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::clusterframes(std::map<int,std::shared_ptr<ClusterFrames<PointType_>>> &_clusterframes){
        this->status("BA","Cleaning old data");
        mUsedWordsMap.clear();
        mClustersIdToCameraId.clear();
        mWordIdToPointId.clear();
        mGlobalUsedWordsRef.clear();
        cleanData();

        mClusterFrames = _clusterframes;
    };

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjuster<PointType_, DebugLevel_, OutInterface_>::prepareDataClusterframes(){
        this->status("BA","Preparing data");
        unsigned nWords = 0;
        for(auto &cluster: mClusterFrames){
            auto bestDataframe = cluster.second->bestDataframePtr();
            for(auto  &word: bestDataframe->wordsReference){
                if(!mUsedWordsMap[word->id] &&  word->clusters.size() > this->mBaMinAparitions){
                    nWords++;
                    mUsedWordsMap[word->id] = true;  // check true to use it later
                    mGlobalUsedWordsRef[word->id] = word;
                }
            }
        }
        
        this->status("BA","Found " + std::to_string(nWords) + " that exists in at least "+std::to_string(this->mBaMinAparitions)+" clusters");
        
        if(nWords < mMinWords){
            this->warning("BA", "Not enough words to perform optimization");
            return false;
        }

        this->status("BA","Copying poses and camera data");
        int nFrames = mClusterFrames.size();
        reserveData(nFrames, nWords);

        int cameraId = 0;
        for(auto &cluster:mClusterFrames){
            cv::Mat intrinsics, coeffs;
            cluster.second->intrinsic.convertTo(intrinsics, CV_64F);
            cluster.second->distCoeff.convertTo(coeffs, CV_64F);
            
            appendCamera(cameraId, cluster.second->pose, intrinsics, coeffs);

            mClustersIdToCameraId[cluster.second->id] = cameraId;
            mCameraIdToClustersId[cameraId] = cluster.second->id;
            
            cameraId++;
        }

        this->status("BA","Copying words' position and projections");
    
        int pointId = 0;
        for(auto &cluster:mClusterFrames){
            cv::Mat intrinsics, coeffs;
            cluster.second->intrinsic.convertTo(intrinsics, CV_64F);
            cluster.second->distCoeff.convertTo(coeffs, CV_64F);
            //int bestDataframeId = cluster.second->bestDataframe;
            for(auto &word: cluster.second->bestDataframePtr()->wordsReference){
                if(!mUsedWordsMap[word->id])
                    continue;

                mUsedWordsMap[word->id] = false; // check false to prevent its use

                appendPoint(pointId, {word->point[0], word->point[1], word->point[2]});
                
                mWordIdToPointId[word->id] = pointId;
                mPointIdToWordId[pointId] = word->id;

                for(auto &clusterId: word->clusters){
                    if(mClustersIdToCameraId.find(clusterId) != mClustersIdToCameraId.end()){
                        auto bestDfIdInCluster = mClusterFrames[clusterId]->bestDataframe;
                        if(word->isInFrame(bestDfIdInCluster)){ // Word can be in cluster but not in best DF of cluster.
                            int cameraId = mClustersIdToCameraId[clusterId];
                            if(word->projectionsEnabled[bestDfIdInCluster])  
                                appendProjection(cameraId, pointId, word->cvProjectiond(bestDfIdInCluster), intrinsics, coeffs);            
                        }
                    }
                }
                pointId++;
            }
        }

        fitSize(mClusterFrames.size(), pointId);

        return true;
    }
    
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjuster<PointType_, DebugLevel_, OutInterface_>:: optimizeClusterframes(){
        this->status("BA","Optimizing " + std::to_string(mClusterFrames.size()) + " cluster frames");
        
        if(!prepareDataClusterframes()){
            this->warning("BA", "Failed data preparation");    
            return false;
        }

        checkData();

        this->status("BA", "Init optimization");
        
        if(!doOptimize()){
            this->error("BA", "Failed Optimization");
            return false;
        }

        this->status("BA", "Optimized, recovering data");

        // Recovering cluster data
        for(auto &pairCamera : mCameraIdToClustersId){
            Eigen::Matrix4f pose;
            cv::Mat intrinsics, coeffs;
            recoverCamera(pairCamera.first, pose, intrinsics, coeffs);

            mClusterFrames[pairCamera.second]->bestDataframePtr()->updatePose(pose);

            // auto cluster = this->mClusterFrames[this->mClustersIdToCameraId[i]]; 
            // Eigen::Matrix4f offsetCluster = cluster->bestDataframePtr()->pose.inverse()*newPose;
            // cluster->bestDataframePtr()->updatePose(newPose);
            // for(auto &df : cluster->dataframes){
            //     if(df.second->id != cluster->bestDataframe){
            //         Eigen::Matrix4f updatedPose = offsetCluster*df.second->pose;
            //         df.second->updatePose(updatedPose);
            //     }
            // }
        }

        // recovering points
        for(auto &pairPoint : mPointIdToWordId){
            cv::Mat intrinsics, coeffs;
            Eigen::Vector3f position;

            recoverPoint(pairPoint.first, position);

            auto word = mGlobalUsedWordsRef[pairPoint.second];
            word->point = {
                position[0],
                position[1],
                position[2]
            };
            word->optimized = true;

            for(auto &clusterId: word->clusters){
                if(mClustersIdToCameraId.find(clusterId) != mClustersIdToCameraId.end()){
                    auto bestDfIdInCluster = mClusterFrames[clusterId]->bestDataframe;
                    if(word->isInFrame(bestDfIdInCluster)){ // Word can be in cluster but not in best DF of cluster.
                        int cameraId = mClustersIdToCameraId[clusterId];
                        if(!isProjectionEnabled(cameraId, pairPoint.first)){
                            mGlobalUsedWordsRef[pairPoint.second]->projectionsEnabled[bestDfIdInCluster] = false;
                            this->warning("BA", "Dropping edge (camera, point): ("+std::to_string(bestDfIdInCluster)+", "+std::to_string(pairPoint.second)+")");
                        }
                    }
                }
            }

        }

        this->status("BA", "Data restored");
        return true;
    }
}
