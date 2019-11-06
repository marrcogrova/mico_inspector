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



#include <mico/flow/blocks/visualizers/BlockSceneVisualizer.h>
#include <mico/flow/Policy.h>
#include <mico/flow/OutPipe.h>


#include <mico/base/map3d/Dataframe.h>

#include <Eigen/Eigen>


namespace mico{

    // Static member initialiation
    bool BlockSceneVisualizer::sAlreadyExisting_ = false;

    BlockSceneVisualizer::BlockSceneVisualizer(){
        if(sAlreadyExisting_)
            return;

        sAlreadyExisting_ = true;
        sBelonger_ = true;

        spinnerThread_ = std::thread([this]{
            cjson::Json configFile;
            configFile["enable"] = true;
            sceneVisualizer_.init(configFile);

            while(run_){
                if(hasPose){
                    // update last pose
                    poseGuard_.lock();
                    Eigen::Matrix4f pose = lastPose_;
                    poseGuard_.unlock();
                    sceneVisualizer_.updateCurrentPose(pose);
                }

                while(queueDfs_.size() > 0){
                    queueGuard_.lock();
                    auto cf = queueDfs_.front();
                    queueDfs_.pop_front();
                    queueGuard_.unlock();
                    sceneVisualizer_.drawDataframe(cf);
                }

                // Check optimizations.
                sceneVisualizer_.checkAndRedrawCf();
                
                // Spin once.
                sceneVisualizer_.spinOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            // Self destroy
            sceneVisualizer_.close();
        });

        iPolicy_ = new Policy({"pose", "dataframe"});

        iPolicy_->registerCallback({"pose" }, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    Eigen::Matrix4f pose = std::any_cast<Eigen::Matrix4f>(_data["pose"]); 
                                    poseGuard_.lock();
                                    lastPose_ = pose;
                                    poseGuard_.unlock();
                                    hasPose = true;
                                }
                            );

        iPolicy_->registerCallback({"dataframe" }, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    Dataframe<pcl::PointXYZRGBNormal>::Ptr df = std::any_cast<Dataframe<pcl::PointXYZRGBNormal>::Ptr>(_data["dataframe"]); 
                                    queueGuard_.lock();
                                    queueDfs_.push_back(df);
                                    queueGuard_.unlock();
                                    idle_ = true;
                                }
                            );
        
    }

    BlockSceneVisualizer::~BlockSceneVisualizer(){
        if(sBelonger_){
            run_ = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if(spinnerThread_.joinable())
                spinnerThread_.join();
            sAlreadyExisting_ = false;
        }
    }

}
