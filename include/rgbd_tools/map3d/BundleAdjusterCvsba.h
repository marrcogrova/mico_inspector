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

#ifndef RGBDTOOLS_MAP3D_BUNDLEADJUSTERCVSBA_H_
#define RGBDTOOLS_MAP3D_BUNDLEADJUSTERCVSBA_H_

#include <rgbd_tools/map3d/DataFrame.h>
#include <rgbd_tools/map3d/BundleAdjuster.h>

namespace rgbd{
    template<typename PointType_>
    class BundleAdjusterCvsba : public BundleAdjuster<PointType_>{
    public:
        bool optimize();
        bool optimizeClusterframes();
        void keyframes(std::vector<std::shared_ptr<DataFrame<PointType_>>> &_keyframes);
        void keyframes(typename std::vector<std::shared_ptr<DataFrame<PointType_>>>::iterator &_begin, typename std::vector<std::shared_ptr<DataFrame<PointType_>>>::iterator &_end);
        void clusterframe(std::shared_ptr<ClusterFrames<PointType_>> &_clusterframe);

        /// \brief Get keyframes. Optimized of optimize() is call and success.
        /// \return internal stored keyframes.
        std::vector<DataFrame<PointType_>, Eigen::aligned_allocator <DataFrame<PointType_>>> keyframes();

    private:
        void cleanData();
        bool prepareData();
        bool prepareDataCluster();

    private:
        std::vector<std::shared_ptr<DataFrame<PointType_>>> mKeyframes;
        std::shared_ptr<ClusterFrames<PointType_>> mClusterframe= nullptr;

        std::vector<cv::Point3d>                mScenePoints;
        std::vector<std::vector<int>>           mCovisibilityMatrix;
        std::vector<std::vector<cv::Point2d>>   mScenePointsProjection;
        std::vector<int> mIdxToId;

        std::vector<cv::Mat> mTranslations, mRotations, mIntrinsics, mCoeffs;

    };
}   // namespace rgbd

#include <rgbd_tools/map3d/BundleAdjusterCvsba.inl>

#endif //RGBDTOOLS_MAP3D_BUNDLEADJUSTERCSVBA_H_
