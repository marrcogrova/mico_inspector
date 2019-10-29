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
        iPolicy_ = new Policy({"pose", "acceleration"});

        opipes_["pose"] = new OutPipe("pose");

        prevT_ = std::chrono::system_clock::now();

        iPolicy_->setCallback({"pose"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    startFilter_ = true;
                                    if(idle_){
                                        idle_ = false;
                                        Eigen::Matrix4f pose = std::any_cast<Eigen::Matrix4f>(_data["pose"]);
                                        Eigen::Vector3f position = pose.block<3,1>(0,3);

                                        printf("position used in EKF: px %f,py %f,pz %f \n",position[0],position[1],position[2]);
                                        // New observation EKF
                                        Eigen::Matrix<double,6,1> Zk = Eigen::Matrix<double,6,1>::Identity();
                                        Zk << double(position[0]) , double(position[1]) , double(position[2]), 0.0 , 0.0 , 0.0;
                                        if (!newObservation(Zk)){
                                            return;
                                        }
                                        idle_ = true;
                                    }       
                                }
        );

        iPolicy_->setCallback({"acceleration"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_ ){
                                        if (!startFilter_){
                                            return;
                                            }
                                        idle_ = false;

                                        Eigen::Vector3f acc = std::any_cast<Eigen::Vector3f>(_data["acceleration"]);
                                        printf(" acceleration used in EKF: ax %f,ay %f,az %f \n",acc[0],acc[1],acc[2]);
                                        // New observation EKF
                                        Eigen::Matrix<double,6,1> Zk = Eigen::Matrix<double,6,1>::Identity();
                                        Zk << 0.0 , 0.0 , 0.0, double(acc[0]) , double(acc[1]) , double(acc[2]);
                                        if (!newObservation(Zk)){
                                            return;
                                        }
                                        idle_ = true;
                                    }   
                                
                                }
        );

        // callback_ = [&](std::unordered_map<std::string,std::any> _data, std::unordered_map<std::string,bool> _valid){
        //     if(idle_){
                
        //         idle_ = false;
        //         auto t1 = std::chrono::system_clock::now();
        //         auto incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-prevT_).count()/1000.0f;
        //         prevT_ = t1;

        //         Eigen::Vector3f acc = std::any_cast<Eigen::Vector3f>(_data["acceleration"]);
        //         Eigen::Quaternionf ori = std::any_cast<Eigen::Quaternionf>(_data["orientation"]);

        //         Eigen::Vector3f linAcc = /*ori**/acc - gravity_;                              
    
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
        std::string path;
        for(auto &param: _params){
            if(param.first =="EKFconfig"){
                path = param.second;
            }
        }
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cout << "Cannot open file." << std::endl;
            return false;
        }
        cjson::Json configFile;
        if (!configFile.parse(file)) {
            std::cout << "Cannot parse config file." << std::endl;
            return false;
        }
        if (configFile.contains("Extended_Kalman_Filter")) {
            if (!ekf_.init(configFile["Extended_Kalman_Filter"])) {
                printf("Error inicializating EKF parameters \n");
                return false;
            }
        }
        printf("EKF parameters inicializated \n");
        return true;
    }
    
    std::vector<std::string> BlockEKFIMU::parameters(){
        return {"EKFconfig"};
    }

    bool BlockEKFIMU::newObservation(Eigen::Matrix<double,6,1> _Zk){
        auto t1 = std::chrono::system_clock::now();
        auto incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1-prevT_).count()/1000.0f;
        // make new observation
        ekf_.stepEKF(_Zk,double(incT)); // double(0.3);
        prevT_ = t1;
        
        // get state vector
        Eigen::Matrix<double,12,1> Xk = ekf_.state();
        Eigen::Matrix4f poseEKF = Eigen::Matrix4f::Identity();
        Eigen::Vector3f xk;
        xk[0] = float(Xk(0,0));
        xk[1] = float(Xk(1,0));
        xk[2] = float(Xk(2,0));
        poseEKF.block<3,1>(0,3) = xk;
        printf("position EKF: x %f,y %f,z %f \n",xk[0],xk[1],xk[2]);
        opipes_["pose"]->flush(poseEKF);

    
        return true;
    }
}
