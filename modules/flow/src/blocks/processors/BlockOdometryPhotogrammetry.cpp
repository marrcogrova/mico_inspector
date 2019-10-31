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

#include <mico/flow/blocks/processors/BlockOdometryPhotogrammetry.h>
#include <mico/flow/Policy.h>
#include <mico/flow/OutPipe.h>

namespace mico{

    BlockOdometryPhotogrammetry::BlockOdometryPhotogrammetry(){
        
        iPolicy_ = new Policy({"color", "altitude", "clusterframe"});

        opipes_["dataframe"] = new OutPipe("dataframe");

        featureDetector_ = cv::ORB::create(1000);
        
        iPolicy_->setCallback({"color", "altitude"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;

                                        altitude_ = std::any_cast<float>(_data["altitude"]);
                                        if (!savedFirstAltitude_){
                                            firstAltitude_ = altitude_;
                                            savedFirstAltitude_ = true;
                                        }
                                        if(hasCalibration){
                                            std::shared_ptr<mico::DataFrame<pcl::PointXYZRGBNormal>> df(new mico::DataFrame<pcl::PointXYZRGBNormal>());
                                            df->id = nextDfId_;
                                            try{
                                                df->left = std::any_cast<cv::Mat>(_data["color"]); 
                                            }catch(std::exception& e){
                                                std::cout << "Failure Odometry Photogrammetry " <<  e.what() << std::endl;
                                                idle_ = true;
                                                return;
                                            }
                                            if(!computePointCloud(df)){
                                                idle_ = true;
                                                return;
                                            }

                                            if(df->featureDescriptors.rows == 0)
                                                return;

                                            if(lastClusterFrame_ != nullptr){
                                                if(odom_.computeOdometry(lastClusterFrame_, df)){
                                                    nextDfId_++;
                                                    opipes_["dataframe"]->flush(df);  
                                                    // std::cout << "using cluster " << std::endl;
                                                }
                                            }else{
                                                if(prevDf_!=nullptr){
                                                    if(odom_.computeOdometry(prevDf_, df)){
                                                        nextDfId_++;
                                                        opipes_["dataframe"]->flush(df);  
                                                        prevDf_ = df;
                                                        // std::cout << "using dataframe " << std::endl;
                                                    }
                                                }else{
                                                    prevDf_ = df;
                                                }
                                            }
                                        }else{
                                            std::cout << "Please, configure Odometry Photogrammetry with the path to the calibration file {\"Calibration\":\"/path/to/file\"}" << std::endl;
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


    bool BlockOdometryPhotogrammetry::configure(std::unordered_map<std::string, std::string> _params){
        for(auto &param: _params){
            if(param.first == "calibration"){

                cv::FileStorage fs(param.second, cv::FileStorage::READ);
                fs["MatrixLeft"]            >> matrixLeft_;
                fs["DistCoeffsLeft"]        >> distCoefLeft_;
                
                hasCalibration = true;
                return true;
            }
        }

        return false;

    }
    
    std::vector<std::string> BlockOdometryPhotogrammetry::parameters(){
        return {"calibration"};
    }

    bool BlockOdometryPhotogrammetry::computePointCloud(std::shared_ptr<mico::DataFrame<pcl::PointXYZRGBNormal>> &_df){
        cv::Mat descriptors;
        std::vector<cv::KeyPoint> kpts;
        cv::Mat leftGrayUndistort;

        cv::cvtColor(_df->left, leftGrayUndistort, cv::ColorConversionCodes::COLOR_BGR2GRAY);
        featureDetector_->detectAndCompute(leftGrayUndistort, cv::Mat(), kpts, descriptors);
        if (kpts.size() < 8) {
            return false;
        }
        _df->featureDescriptors = descriptors;
        
        // bad SLAM inicialization
        float _altitude = (altitude_ - firstAltitude_);
        if (_altitude < initSLAMAltitude_ ){
            printf("Actual altitude  %f m, SLAM inicializate when %f m \n",_altitude, initSLAMAltitude_);
            return false;
        }
        // Create feature cloud
        _df->featureCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        _df->cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        if(pinHoleModel(_altitude,kpts, _df->featureCloud)){
            _df->featureProjections.resize(kpts.size());
            for (unsigned k = 0; k < kpts.size(); k++) {
                _df->featureProjections[k] = kpts[k].pt;
            }
            pcl::copyPointCloud(*(_df->featureCloud) , *(_df->cloud));
        }else{
            return false;
        }
        // Filling new dataframe
        _df->orientation = Eigen::Matrix3f::Identity();
        _df->position = Eigen::Vector3f::Zero();

        return true;
    }


    bool BlockOdometryPhotogrammetry::pinHoleModel(float _altitude ,std::vector<cv::KeyPoint> keypoints, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr OutputPointCloud){
        const float cx = matrixLeft_.at<float>(0,2);
        const float cy = matrixLeft_.at<float>(1,2);    // 666 Move to member? faster method....
        const float fx = matrixLeft_.at<float>(0,0);
        const float fy = matrixLeft_.at<float>(1,1);
        
        for(unsigned ii = 0; ii < keypoints.size(); ii++){
            pcl::PointXYZRGBNormal p;
            p.z = - (_altitude) ;
            p.x = -( ( keypoints[ii].pt.x - cx )/fx ) * (-p.z);
            p.y =  ( ( keypoints[ii].pt.y - cy )/fy ) * (-p.z);
            p.r = 255; p.g = 255; p.b = 255;
    
            OutputPointCloud->points.push_back(p);
        }
        if (OutputPointCloud->points.size() == 0){
            return false;
        }
        return true;
    }
}

