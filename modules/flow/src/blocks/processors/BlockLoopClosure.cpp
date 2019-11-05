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
        iPolicy_ = new Policy({"dataframe"});

        opipes_["v-dataframe"] = new OutPipe("v-dataframe");
        
        iPolicy_->registerCallback({"dataframe"}, 
                                [&](std::unordered_map<std::string,std::any> _data){
                                    if(idle_){
                                        idle_ = false;
                                        ClusterFrames<pcl::PointXYZRGBNormal>::Ptr cf = std::any_cast<ClusterFrames<pcl::PointXYZRGBNormal>::Ptr>(_data["dataframe"]); 
                                        
                                        LoopResult res = loopDetector_.appendCluster(cf->left, cf->id);
                                        dataframes_[cf->id] = cf;

                                        if(res.found){ // New dataframe created 
                                            std::cout << "Detected loop... WIP parse loop" << std::endl;
                                            cf->isOptimized(true);  // 666 Mark as optimized to be redrawn

                                            //  vvvvvvv  ALL this shit happened together in SLAM_MARK_I vvvvvvv
                                            //
                                            // std::map<int,std::shared_ptr<ClusterFrames<PointType_>>> loopClosureSubset;
                                            // loopClosureSubset[mDatabase.mLastClusterframe->id] = mDatabase.mLastClusterframe;
                                            // loopClosureSubset[result.matchId] = mDatabase.mClusterframes[result.matchId];
                                            // mDatabase.dfComparison(loopClosureSubset, false);

                                            // mVisualization->drawDataframe(mDatabase.mLastClusterframe);
                                            // mVisualization->drawDataframe(mDatabase.mClusterframes[result.matchId]);
                                            //
                                            //  ^^^^^^^^ --------------------------------------------- ^^^^^^^^ 
                                            
                                            // opipes_["v-dataframe"]->flush();
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
