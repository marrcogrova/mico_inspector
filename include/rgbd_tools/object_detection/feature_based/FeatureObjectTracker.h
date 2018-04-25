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



#ifndef RGBDTOOLS_OBJECTDETECTION_FEATUREBASED_FEATUREOJECTTRACKER_H_
#define RGBDTOOLS_OBJECTDETECTION_FEATUREBASED_FEATUREOJECTTRACKER_H_

#include <chrono>

#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>

#include <rgbd_tools/cjson/json.h>
#include <rgbd_tools/object_detection/feature_based/FeatureModel.h>
#include <rgbd_tools/object_detection/feature_based/SimpleKinematicEKF.h>

namespace rgbd{
    class FeatureObjectTracker{
    public:
        bool init(cjson::Json &_config);

        bool update(cv::Mat &_image, cv::Mat &_position, cv::Mat &_orientation);

        void drawCoordinate(const cv::Mat &_position, const cv::Mat &_rotation, cv::Mat &_image);
        void drawCurrentWindow(cv::Mat &_image);
    private:
        void computeNextWindow(const std::vector<cv::Point2f> &_points);
        void increaseSearchWindow(int _width, int _height);
    
    private:
        enum class AppStatus {Lost, Found};
        AppStatus mStatus = AppStatus::Lost;

        cv::Rect mLastWindow;
        unsigned mNumLostFrames = 0;

        FeatureModel mModel;

        unsigned mMaxLostFrames;
        double mScaleFactorWindowLost;

        SimpleKinematicEKF mEKF;
        Eigen::MatrixXd mQ, mR;
        std::chrono::time_point<std::chrono::high_resolution_clock> mTimeStamp;

        cv::Mat mIntrinsics, mDistCoeff;
    };
}

#endif