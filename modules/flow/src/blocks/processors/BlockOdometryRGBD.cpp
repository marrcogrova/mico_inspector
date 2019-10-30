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

#include <mico/flow/blocks/processors/BlockOdometryRGBD.h>
#include <mico/flow/Policy.h>
#include <mico/flow/OutPipe.h>

namespace mico{

    BlockOdometryRGBD::BlockOdometryRGBD(){
        
        iPolicy_ = new Policy({"color", "depth", "cloud", "clusterframe"});

        opipes_["dataframe"] = new OutPipe("dataframe");

        featureDetector_ = cv::ORB::create(1000);
        
        iPolicy_->setCallback({"color", "depth", "cloud"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;
                                        if(hasCalibration){
                                            std::shared_ptr<mico::DataFrame<pcl::PointXYZRGBNormal>> df(new mico::DataFrame<pcl::PointXYZRGBNormal>());
                                            df->id = nextDfId_;
                                            try{
                                                df->left = std::any_cast<cv::Mat>(_data["color"]);
                                                df->depth = std::any_cast<cv::Mat>(_data["depth"]);
                                                df->cloud = std::any_cast<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>(_data["cloud"]);   
                                                df->intrinsic = matrixLeft_;
                                                df->coefficients = distCoefLeft_;
                                            }catch(std::exception& e){
                                                std::cout << "Failure OdometryRGBD. " <<  e.what() << std::endl;
                                                idle_ = true;
                                                return;
                                            }
                                            computeFeatures(df);

                                            if(df->featureDescriptors.rows == 0)
                                                return;

                                            if(lastClusterFrame_ != nullptr){
                                                 if(odom_.computeOdometry(lastClusterFrame_, df)){
                                                    nextDfId_++;
                                                    opipes_["dataframe"]->flush(df);  
                                                    std::cout << "using cluster " << std::endl;
                                                }
                                            }else{
                                                if(prevDf_!=nullptr){
                                                    if(odom_.computeOdometry(prevDf_, df)){
                                                        nextDfId_++;
                                                        opipes_["dataframe"]->flush(df);  
                                                        prevDf_ = df;
                                                    }
                                                }else{
                                                    prevDf_ = df;
                                                }
                                            }
                                        }else{
                                            std::cout << "Please, configure Odometry RGBD with the path to the calibration file {\"Calibration\":\"/path/to/file\"}" << std::endl;
                                        }
                                        idle_ = true;
                                    }
                                });
        iPolicy_->setCallback({"clusterframe"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                        lastClusterFrame_ = std::any_cast<std::shared_ptr<mico::ClusterFrames<pcl::PointXYZRGBNormal>>>(_data["clusterframe"]);
                                    }
                                );

    }


    bool BlockOdometryRGBD::configure(std::unordered_map<std::string, std::string> _params){
        for(auto &param: _params){
            if(param.first == "calibration"){

                cv::FileStorage fs(param.second, cv::FileStorage::READ);

                fs["MatrixLeft"]            >> matrixLeft_;
                fs["DistCoeffsLeft"]        >> distCoefLeft_;
                fs["MatrixRight"]           >> matrixRight_;
                fs["DistCoeffsRight"]       >> distCoefRight_;
                fs["DisparityToDepthScale"] >> dispToDepth_;
                
                hasCalibration = true;
                return true;
            }
        }

        return false;

    }
    
    std::vector<std::string> BlockOdometryRGBD::parameters(){
        return {"calibration"};
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
                [[maybe_unused]] float dist = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
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
        const float cx = matrixLeft_.at<float>(0,2);
        const float cy = matrixLeft_.at<float>(1,2);    // 666 Move to member? faster method....
        const float fx = matrixLeft_.at<float>(0,0);
        const float fy = matrixLeft_.at<float>(1,1);
        const float mDispToDepth = dispToDepth_;

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
