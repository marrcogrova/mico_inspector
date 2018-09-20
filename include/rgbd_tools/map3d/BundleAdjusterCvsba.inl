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


#include <rgbd_tools/map3d/cvsba/cvsba.h>
#include <unordered_map>
#include <rgbd_tools/utils/Gui.h>
#include <pcl/common/transforms.h>
#include <algorithm>
#include <iterator>

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::cleanData() {
        mCovisibilityMatrix.clear();
        mScenePoints.clear();
        mScenePointsProjection.clear();
        mIntrinsics.clear();
        mCoeffs.clear();
        mTranslations.clear();
        mRotations.clear();
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::reserveData(int _cameras, int _words){
        mCovisibilityMatrix.resize(_cameras);
        mScenePointsProjection.resize(_cameras);
        mRotations.resize(_cameras);
        mTranslations.resize(_cameras);
        mIntrinsics.resize(_cameras);
        mCoeffs.resize(_cameras);

        mScenePoints.resize(_words);
        for(int i = 0; i < _cameras; i++){
            mCovisibilityMatrix[i].resize(_words, 0);
            mScenePointsProjection[i].resize(   _words, 
                                                cv::Point2d(    std::numeric_limits<double>::quiet_NaN(),
                                                                std::numeric_limits<double>::quiet_NaN()));
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::appendCamera(int _id, Eigen::Matrix4f _pose, cv::Mat _intrinsics, cv::Mat _distcoeff){
        Eigen::Matrix4f poseInv = _pose.inverse();

        mIntrinsics[_id] = _intrinsics.clone();
        mCoeffs[_id] = _distcoeff.clone();

        cv::Mat cvRotation(3,3,CV_64F);
        cvRotation.at<double>(0,0) = poseInv(0,0);
        cvRotation.at<double>(0,1) = poseInv(0,1);
        cvRotation.at<double>(0,2) = poseInv(0,2);
        cvRotation.at<double>(1,0) = poseInv(1,0);
        cvRotation.at<double>(1,1) = poseInv(1,1);
        cvRotation.at<double>(1,2) = poseInv(1,2);
        cvRotation.at<double>(2,0) = poseInv(2,0);
        cvRotation.at<double>(2,1) = poseInv(2,1);
        cvRotation.at<double>(2,2) = poseInv(2,2);
        mRotations[_id] = cvRotation.clone();

        cv::Mat cvTrans(3,1,CV_64F);
        cvTrans.at<double>(0) = poseInv(0,3);
        cvTrans.at<double>(1) = poseInv(1,3);
        cvTrans.at<double>(2) = poseInv(2,3);
        mTranslations[_id] = cvTrans.clone();
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::appendPoint(int _id, Eigen::Vector3f _position){
        mScenePoints[_id] = cv::Point3d(_position[0], _position[1], _position[2]);
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::appendProjection(int _idCamera, int _idPoint, cv::Point2f _projection, cv::Mat _intrinsics, cv::Mat _distcoeff){
        mScenePointsProjection[_idCamera][_idPoint] = _projection;
        mCovisibilityMatrix[_idCamera][_idPoint] = 1;
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::fitSize(int _cameras, int _words){
        mScenePoints.resize(_words);
        for(auto&visibility:mCovisibilityMatrix){
            visibility.resize(_words);
        }
        for(auto&projections:mScenePointsProjection){
            projections.resize(_words);
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::checkData(){
        assert(mScenePoints.size() == mScenePointsProjection[0].size());
        assert(mCovisibilityMatrix[0].size() == mScenePoints.size());
        assert(mCovisibilityMatrix.size() == mScenePointsProjection.size());
        assert(mIntrinsics.size() == mScenePointsProjection.size());
        assert(mIntrinsics.size() == mCoeffs.size());
        assert(mIntrinsics.size() == mRotations.size());
        assert(mTranslations.size() == mRotations.size());
    }



    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::doOptimize(){
        // Initialize cvSBA and perform bundle adjustment.
        cvsba::Sba bundleAdjuster;
        cvsba::Sba::Params params;
        params.verbose = true;
        params.iterations = this->mBaIterations;
        params.minError = this->mBaMinError;
        params.type = cvsba::Sba::MOTIONSTRUCTURE;
        bundleAdjuster.setParams(params);

        try{
            bundleAdjuster.run(mScenePoints, mScenePointsProjection, mCovisibilityMatrix, mIntrinsics, mRotations, mTranslations, mCoeffs);
        }catch(cv::Exception &_e){
            std::cout << _e.what() << std::endl;
            return false;
        }

        return true;
    }


    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::recoverCameras(){
        Eigen::Matrix4f pose01 = this->mClusterFrames[this->mClustersIdxToId[0]]->bestDataframePtr()->pose;
        Eigen::Matrix4f incPose = Eigen::Matrix4f::Identity();
        for(unsigned i = 0; i < mTranslations.size(); i++){
            Eigen::Matrix4f newPose = Eigen::Matrix4f::Identity();
            
            newPose(0,0) = mRotations[i].at<double>(0,0);
            newPose(0,1) = mRotations[i].at<double>(0,1);
            newPose(0,2) = mRotations[i].at<double>(0,2);
            newPose(1,0) = mRotations[i].at<double>(1,0);
            newPose(1,1) = mRotations[i].at<double>(1,1);
            newPose(1,2) = mRotations[i].at<double>(1,2);
            newPose(2,0) = mRotations[i].at<double>(2,0);
            newPose(2,1) = mRotations[i].at<double>(2,1);
            newPose(2,2) = mRotations[i].at<double>(2,2);
            
            newPose(0,3) = mTranslations[i].at<double>(0);
            newPose(1,3) = mTranslations[i].at<double>(1);
            newPose(2,3) = mTranslations[i].at<double>(2);
            
            newPose = newPose.inverse().eval();

            if(i == 0){
                Eigen::Matrix4f pose02 = newPose;
                incPose = pose02.inverse()*pose01;
            }

            newPose = incPose*newPose;

            auto cluster = this->mClusterFrames[this->mClustersIdxToId[i]]; 

            Eigen::Matrix4f offsetCluster = cluster->bestDataframePtr()->pose.inverse()*newPose;
            
            cluster->bestDataframePtr()->updatePose(newPose);

            for(auto &df : cluster->dataframes){
                if(df.second->id != cluster->bestDataframe){
                    Eigen::Matrix4f updatedPose = offsetCluster*df.second->pose;
                    df.second->updatePose(updatedPose);
                }
            }
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::recoverPoints(){
        for(unsigned i = 0; i < this->mWordIdxToId.size(); i++){
            int id = this->mWordIdxToId[i];
            this->mGlobalUsedWordsRef[id]->point = {
                (float) mScenePoints[i].x,
                (float) mScenePoints[i].y,
                (float) mScenePoints[i].z
            };
            this->mGlobalUsedWordsRef[id]->optimized = true;
        }
    }
}
