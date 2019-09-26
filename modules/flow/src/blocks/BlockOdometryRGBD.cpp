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

namespace mico{

    BlockOdometryRGBD::BlockOdometryRGBD(){
        // cjson::Json jParams; // 666 Not needed. May be necessary to set a generic configurator in blocks
        // odom_.init(jParams);
        callback_ = [&](std::unordered_map<std::string,std::any> _data, std::unordered_map<std::string,bool> _valid){
            std::shared_ptr<mico::DataFrame<pcl::PointXYZRGBNormal>> df(new mico::DataFrame<pcl::PointXYZRGBNormal>());
            df->id = nextDfId_;
            df->left = std::any_cast<cv::Mat>(_data["rgb"]);
            df->depth = std::any_cast<cv::Mat>(_data["depth"]);
            df->cloud = std::any_cast<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>(_data["cloud"]);   

            cv::imshow("displa", df->left);
            cv::waitKey(3);

            if(hasPrev_){
                if(odom_.computeOdometry(prevDf_, df)){
                    // update
                }
            }else{
                prevDf_ = df;      
                hasPrev_ = true;
            }
        };


        setPolicy(new PolicyAllRequired()); // 666 OH SHIT THE ORDER IS IMPORTANT!

        iPolicy_->setupStream("rgb");
        iPolicy_->setupStream("depth");
        iPolicy_->setupStream("cloud");
    }

}
