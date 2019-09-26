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

#ifndef MICO_BASE_MAP3D_BUNDLEADJUSTERCVSBA_H_
#define MICO_BASE_MAP3D_BUNDLEADJUSTERCVSBA_H_

#include <mico/base/map3d/DataFrame.h>
#include <mico/base/map3d/BundleAdjuster.h>
#include <mico/base/map3d/Word.h>

namespace mico {
  template <typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Null>
    class BundleAdjusterCvsba : public BundleAdjuster<PointType_, DebugLevel_, OutInterface_>{
    public:
    
    protected:
        virtual void appendCamera(int _id, Eigen::Matrix4f _pose, cv::Mat _intrinsics = cv::Mat(), cv::Mat _distcoeff = cv::Mat());

        virtual void appendPoint(int _id, Eigen::Vector3f _position);

        virtual void appendProjection(int _idCamera, int _idPoint, cv::Point2f _projection, cv::Mat _intrinsics = cv::Mat(), cv::Mat _distcoeff = cv::Mat());

        virtual void reserveData(int _cameras, int _words);

        virtual void fitSize(int _cameras, int _words);

        virtual void cleanData();

        virtual void checkData();

        virtual bool doOptimize();

        virtual void recoverCamera(int _id, Eigen::Matrix4f &_pose, cv::Mat &_intrinsics, cv::Mat &_distcoeff);
        virtual void recoverPoint(int _id, Eigen::Vector3f &_position);
        virtual bool isProjectionEnabled(int _idCamera, int _idPoint);

    protected:
        std::vector<std::shared_ptr<DataFrame<PointType_>>> mKeyframes;
        std::shared_ptr<ClusterFrames<PointType_>> mClusterframe= nullptr;

        std::vector<cv::Point3d>                mScenePoints;
        std::vector<std::vector<int>>           mCovisibilityMatrix;
        std::vector<std::vector<cv::Point2d>>   mScenePointsProjection;

        std::vector<cv::Mat> mTranslations, mRotations, mIntrinsics, mCoeffs;
        Eigen::Matrix4f mIncPose01 = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f mPose01 = Eigen::Matrix4f::Identity();
    };
}   // namespace mico 

#include <mico/base/map3d/BundleAdjusterCvsba.inl>

#endif //RGBDTOOLS_MAP3D_BUNDLEADJUSTERCSVBA_H_
