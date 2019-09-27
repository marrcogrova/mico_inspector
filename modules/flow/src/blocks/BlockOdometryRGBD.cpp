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

#include <mico/flow/blocks/BlockOdometryRGBD.h>
#include <mico/flow/policies/policies.h>
#include <mico/flow/streamers/StreamPose.h>

namespace mico{

    BlockOdometryRGBD::BlockOdometryRGBD(){
        // cjson::Json jParams; // 666 Not needed. May be necessary to set a generic configurator in blocks
        // odom_.init(jParams);
        callback_ = [&](std::unordered_map<std::string,std::any> _data, std::unordered_map<std::string,bool> _valid){
            if(idle_){
                idle_ = false;
                std::shared_ptr<mico::DataFrame<pcl::PointXYZRGBNormal>> df(new mico::DataFrame<pcl::PointXYZRGBNormal>());
                df->id = nextDfId_;
                df->left = std::any_cast<cv::Mat>(_data["rgb"]);
                df->depth = std::any_cast<cv::Mat>(_data["depth"]);
                df->cloud = std::any_cast<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>(_data["cloud"]);   
                computeFeatures(df);

                if(df->featureDescriptors.rows == 0)
                    return;

                if(hasPrev_){
                    if(odom_.computeOdometry(prevDf_, df)){
                        std::unordered_map<std::string, std::any> data;
                        data["pose"] = (Eigen::Matrix4f) df->pose;
                        ostreams_["pose"]->manualUpdate(data);
                    }
                }else{      
                    hasPrev_ = true;
                }
                prevDf_ = df;
                idle_ = true;
            }
        };

        ostreams_["pose"] = new StreamPose();

        featureDetector_ = cv::ORB::create(1000);
        setPolicy(new PolicyAllRequired()); // 666 OH SHIT THE ORDER IS IMPORTANT!

        iPolicy_->setupStream("rgb");
        iPolicy_->setupStream("depth");
        iPolicy_->setupStream("cloud");
    }

    void BlockOdometryRGBD::computeFeatures(std::shared_ptr<mico::DataFrame<pcl::PointXYZRGBNormal>> &_df){
        cv::Mat descriptors;
        std::vector<cv::KeyPoint> kpts;
        cv::Mat leftGrayUndistort;

        cv::cvtColor(_df->left, leftGrayUndistort, cv::ColorConversionCodes::COLOR_BGR2RGB);
        featureDetector_->detectAndCompute(leftGrayUndistort, cv::Mat(), kpts, descriptors);
        if (kpts.size() < 8) {
            return;
        }

        // Create feature cloud.
        _df->featureCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        for (unsigned k = 0; k < kpts.size(); k++) {
            cv::Point3f point;
            if (colorPixelToPoint(_df->depth, kpts[k].pt, point)) { // Using coordinates of distorted points to match depth 
                float dist = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
                // if (!std::isnan(point.x) && dist > 0.25 && dist < 6.0) { // 666 min and max dist? 
                    pcl::PointXYZRGBNormal pointpcl;
                    pointpcl.x = point.x;
                    pointpcl.y = point.y;
                    pointpcl.z = point.z;
                    pointpcl.r = 255;
                    pointpcl.g = 0;
                    pointpcl.b = 0;
                    _df->featureCloud->push_back(pointpcl);
                    _df->featureDescriptors.push_back(descriptors.row(k)); // 666 TODO: filter a bit?
                    _df->featureProjections.push_back(kpts[k].pt);    //  Store undistorted points
                //}
            }
        }

        // Filling new dataframe
        _df->orientation = Eigen::Matrix3f::Identity();
        _df->position = Eigen::Vector3f::Zero();
    }

    bool BlockOdometryRGBD::colorPixelToPoint(const cv::Mat &_depth, const cv::Point2f &_pixel, cv::Point3f &_point){
        const float cx = 318.6;  /// 666 SHIT! need to load it too.
        const float cy = 255.3;
        const float fx = 516.5;
        const float fy = 517.3;
        const float mDispToDepth = 0.001;

        // Retrieve the 16-bit depth value and map it into a depth in meters
        uint16_t depth_value = _depth.at<uint16_t>(_pixel.y, _pixel.x);
        float depth_in_meters = depth_value * mDispToDepth;
        // Set invalid pixels with a depth value of zero, which is used to indicate no data
        if (depth_value == 0) {
            return false;
        }
        else {
            // 666 Assuming that it is undistorted which is for intel real sense F200 and depth is in color CS...
            _point.x = (_pixel.x - cx)/fx*depth_in_meters;
            _point.y = (_pixel.y - cy)/fy*depth_in_meters;
            _point.z = depth_in_meters;
            return true;
        }
    }

}
