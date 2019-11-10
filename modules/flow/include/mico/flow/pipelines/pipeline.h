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


#ifndef MICO_FLOW_STREAMERS_PIPELINES_PIPELINE_H_
#define MICO_FLOW_STREAMERS_PIPELINES_PIPELINE_H_
/*
#include <mico/flow/streamers/streamers.h>
#include <mico/flow/blocks/block.h>
#include <mico/flow/policies/policies.h>

namespace mico{
    class Pipeline{
    public:
        template<typename... Ts> 
        void build(Ts... args){
            ((void) builder_expansion(std::forward<Ts>(args)), ...);

            connectBlocks();
        }

        void connectBlocks(){
            for(auto &block: blocks_){
                block->setPolicy(policy_);
            }
            for(auto &osi: streams_){
                osi->registerPolicy(policy_, 0); 
                assert(false);      // HOW TO CHOOSE STREAM!
            }
        }

        void start(){
            for(auto &osi: streams_){
                osi->start();
            }
        }

        void stop(){
            for(auto &osi: streams_){
                osi->stop();
            }
        }

        template<typename T>
        void builder_expansion(T t) {
            // if(typeid(t) == typeid(std::string) || typeid(t) == typeid(char*)){
                if(std::string(t) == "mono"){
                    streams_.push_back(new OstreamCamera());
                }else if(std::string(t) == "all_policy"){
                    policy_ = new flow::PolicyAllRequired();
                }else if(std::string(t) == "odometry"){
                    // block
                }else if(std::string(t) == "visualization"){
                    // block   
                }
            // }
        }

    private:

        std::vector<Ostream*> streams_;
        Policy * policy_;
        std::vector<Block*> blocks_;

    };


    template <>
    void Pipeline::builder_expansion<Ostream*>(Ostream *t) {
        streams_.push_back(t);
    }

    template <>
    void Pipeline::builder_expansion<Block*>(Block *t) {
        blocks_.push_back(t);
    }

}
*/


#endif