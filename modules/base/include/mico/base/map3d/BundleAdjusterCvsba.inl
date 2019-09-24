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


#include <mico/base/map3d/cvsba/cvsba.h>
#include <unordered_map>
#include <mico/base/utils/Gui.h>
#include <pcl/common/transforms.h>
#include <algorithm>
#include <iterator>

namespace mico {
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
        Eigen::Matrix4f cvPose = _pose.inverse();

        if(_id == 0){
            mPose01 = cvPose;
        }

        mIntrinsics[_id] = _intrinsics.clone();
        mCoeffs[_id] = _distcoeff.clone();

        cv::Mat cvRotation(3,3,CV_64F);
        cvRotation.at<double>(0,0) = cvPose(0,0);
        cvRotation.at<double>(0,1) = cvPose(0,1);
        cvRotation.at<double>(0,2) = cvPose(0,2);
        cvRotation.at<double>(1,0) = cvPose(1,0);
        cvRotation.at<double>(1,1) = cvPose(1,1);
        cvRotation.at<double>(1,2) = cvPose(1,2);
        cvRotation.at<double>(2,0) = cvPose(2,0);
        cvRotation.at<double>(2,1) = cvPose(2,1);
        cvRotation.at<double>(2,2) = cvPose(2,2);
        mRotations[_id] = cvRotation.clone();

        cv::Mat cvTrans(3,1,CV_64F);
        cvTrans.at<double>(0) = cvPose(0,3);
        cvTrans.at<double>(1) = cvPose(1,3);
        cvTrans.at<double>(2) = cvPose(2,3);
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
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::recoverCamera(int _id, Eigen::Matrix4f &_pose, cv::Mat &_intrinsics, cv::Mat &_distcoeff){
        
        Eigen::Matrix4f newPose = Eigen::Matrix4f::Identity();
            
        newPose(0,0) = mRotations[_id].at<double>(0,0);
        newPose(0,1) = mRotations[_id].at<double>(0,1);
        newPose(0,2) = mRotations[_id].at<double>(0,2);
        newPose(1,0) = mRotations[_id].at<double>(1,0);
        newPose(1,1) = mRotations[_id].at<double>(1,1);
        newPose(1,2) = mRotations[_id].at<double>(1,2);
        newPose(2,0) = mRotations[_id].at<double>(2,0);
        newPose(2,1) = mRotations[_id].at<double>(2,1);
        newPose(2,2) = mRotations[_id].at<double>(2,2);
        
        newPose(0,3) = mTranslations[_id].at<double>(0);
        newPose(1,3) = mTranslations[_id].at<double>(1);
        newPose(2,3) = mTranslations[_id].at<double>(2);
        
        if(_id == 0){
            mIncPose01 = newPose.inverse()*mPose01;
        }
        
        _pose = mIncPose01*newPose;

        _pose = _pose.inverse().eval();

    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::recoverPoint(int _id, Eigen::Vector3f &_position){
        _position = {
                (float) mScenePoints[_id].x,
                (float) mScenePoints[_id].y,
                (float) mScenePoints[_id].z
            };
    }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline bool BundleAdjusterCvsba<PointType_, DebugLevel_, OutInterface_>::isProjectionEnabled(int _idCamera, int _idPoint){
        return true;
    }

}
