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

#include <mico/flow/blocks/processors/BlockLoopClosure.h>
#include <mico/flow/Policy.h>
#include <mico/flow/OutPipe.h>

#include <sstream>

namespace mico{

    BlockLoopClosure::BlockLoopClosure(){
        iPolicy_ = new Policy({"clusterframe"});

        opipes_["v-clusterframe"] = new OutPipe("v-clusterframe");
        
        iPolicy_->registerCallback({"clusterframe"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;
                                        ClusterFrames<pcl::PointXYZRGBNormal>::Ptr cf = std::any_cast<ClusterFrames<pcl::PointXYZRGBNormal>::Ptr>(_data["clusterframe"]); 
                                        
                                        LoopResult res = loopDetector_.appendCluster(cf->left, cf->id);
                                        clusterframes_[cf->id] = cf;

                                        if(res.found){ // New cluster created 
                                            std::cout << "Detected loop... WIP parse loop" << std::endl;
                                            // opipes_["v-clusterframe"]->flush();
                                        }
                                        idle_ = true;
                                    }
                                }
        );


    }

    BlockLoopClosure::~BlockLoopClosure(){

    } 


    bool BlockLoopClosure::configure(std::unordered_map<std::string, std::string> _params){
        cjson::Json jParams;
        for(auto &param: _params){
            if(param.first =="vocabulary"){
                jParams["vocabulary"] = param.second;
            }
        }

        return loopDetector_.init(jParams);
    }
    
    std::vector<std::string> BlockLoopClosure::parameters(){
        return {"vocabulary"};
    }
}
