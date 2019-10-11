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

#include <mico/flow/blocks/processors/BlockDarknet.h>
#include <mico/flow/Policy.h>
#include <mico/flow/OutPipe.h>

namespace mico{

    BlockDarknet::BlockDarknet(){
        
        iPolicy_ = new Policy({"color", "depth", "cloud", "clusterframe"});

        opipes_["dataframe"] = new OutPipe("dataframe");
        
        iPolicy_->setCallback({"color", "depth", "cloud"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;
                                        if(hasCalibration){
                                        }else{
                                        }
                                        idle_ = true;
                                    }
                                });
        iPolicy_->setCallback({"clusterframe"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                        lastClusterFrame_ = std::any_cast<std::shared_ptr<mico::ClusterFrames<pcl::PointXYZRGBNormal>>>(lastClusterFrame_);
                                    }
                                );

    }


    bool BlockDarknet::configure(std::unordered_map<std::string, std::string> _params){
        for(auto &param: _params){
            if(param.first == "calibration"){
                
                return true;
            }
        }

        return false;

    }
    
    std::vector<std::string> BlockDarknet::parameters(){
        return {"calibration"};
    }


}
