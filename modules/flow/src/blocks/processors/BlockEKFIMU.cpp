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

#include <mico/flow/blocks/processors/BlockEKFIMU.h>
#include <mico/flow/Policy.h>
#include <mico/flow/OutPipe.h>
// #include <mico/flow/streamers/StreamPose.h>
// #include <mico/flow/streamers/StreamDataframe.h>

namespace mico{

    BlockEKFIMU::BlockEKFIMU(){
        iPolicy_ = new Policy({"position", "acceleration"});

        opipes_["position"] = new OutPipe("position");

        // ekf_.setUpEKF(ekf_.Q_, ekf_.R_, ekf_.X0_);
        // ekf_.parameters({ 0,0,0 }, {-1,-1,-1}, x0.block<3,1>(9,0), {0.3,0.7,0.5});
        prevT = std::chrono::system_clock::now();

        iPolicy_->setCallback({"position"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;

                                        //

                                        idle_ = true;
                                    }       
                                }
        );

        iPolicy_->setCallback({"acceleration"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;

                                        //

                                        idle_ = true;
                                    }   
                                
                                }
        );

        // callback_ = [&](std::unordered_map<std::string,std::any> _data, std::unordered_map<std::string,bool> _valid){
        //     if(idle_){
                
        //         idle_ = false;
        //         auto t1 = std::chrono::system_clock::now();
        //         auto incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-prevT).count()/1000.0f;
        //         prevT = t1;

        //         Eigen::Vector3f acc = std::any_cast<Eigen::Vector3f>(_data["acceleration"]);
        //         Eigen::Quaternionf ori = std::any_cast<Eigen::Quaternionf>(_data["orientation"]);

        //         Eigen::Vector3f linAcc = /*ori**/acc - gravity_;

        //         //     // New observation EKF
        //         // Eigen::Matrix<double,6,1> Zk = Eigen::Matrix<double,6,1>::Identity();
        //         // Zk << double(OdomPose_(0,3)),double(OdomPose_(1,3)),double(OdomPose_(2,3)), // VO position
        //         //       ImuAcceleration_(0), ImuAcceleration_(1), ImuAcceleration_(2); // IMU data                                    
    
        //         ekf_.stepEKF(linAcc, double(0.3));  // Fixed time

        //         // auto state = ekf_.state();
        //         // std::cout << incT << std::endl;
        //         // std::cout << ori.matrix() <<std::endl;
        //         // std::cout << linAcc.transpose()<<std::endl;
        //         // std::cout << state.transpose() << std::endl;
        //         // Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        //         // pose.block<3,1>(0,3) = state.head(3);

        //         // std::unordered_map<std::string, std::any> data;
        //         // data["pose"] = pose;
        //         // ostreams_["pose"]->manualUpdate(data);
                
        //         idle_ = true;
        //     }
        // };
    }

    bool BlockEKFIMU::configure(std::unordered_map<std::string, std::string> _params){
        cjson::Json jParams;
        for(auto &param: _params){
            if(param.first =="EKFconfig"){
                jParams["Extended_Kalman_Filter"] = param.second;
            }
        }
        return ekf_.init(jParams);
    }
    
    std::vector<std::string> BlockEKFIMU::parameters(){
        return {"EKFconfig"};
    }
}
