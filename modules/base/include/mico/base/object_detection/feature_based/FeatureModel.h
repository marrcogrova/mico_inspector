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



#ifndef MICO_BASE_OBJECTDETECTION_FEATUREBASED_FEATUREMODEL_H_
#define MICO_BASE_OBJECTDETECTION_FEATUREBASED_FEATUREMODEL_H_

#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Eigen>
#include <mico/base/object_detection/feature_based/ImageFeaturesManager.h>

namespace mico {
    class FeatureModel{
    public:
        bool init(const cjson::Json &_configuration);

        /// save the model in xml format.
        /// \param _modelName: name of file without .xml
        bool save(std::string _modelName);

        /// load model from a file
        /// \param _modelName: name of file without .xml
        bool load(std::string _modelName);

        bool find(cv::Mat &_image,const cv::Mat &_intrinsic, const cv::Mat &_coeff,  cv::Mat &_position, cv::Mat &_orientation, std::vector<cv::Point2f> &_inliers, bool _useGuess = false, cv::Rect _roi = cv::Rect(0,0,-1,-1));

        /// 666 TODO: move to private
        // Model variables
        std::vector<cv::Point3f>                mPoints;
        std::vector<std::vector<cv::Point2f>>   mProjections;
        std::vector<std::vector<int>>           mVisibility;
        std::vector<cv::Mat>                    mRs, mTs;
        cv::Mat                                 mDescriptors;

        // Feature detector variables and config
        ImageFeatureManager                     mImageFeatureManager;

        // Ransac variables and config
        
        unsigned mRansacIterations = 3000, mInliersThreshold =12;
        double mRansacErr = 3.0, mRansacConf = 0.97;
    };
}

#endif	
