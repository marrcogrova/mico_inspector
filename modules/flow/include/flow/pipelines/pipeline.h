

#ifndef MICO_FLOW_STREAMERS_PIPELINES_PIPELINE_H_
#define MICO_FLOW_STREAMERS_PIPELINES_PIPELINE_H_

#include <mico/flow/streamers.h>
#include <mico/flow/block.h>


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
            osi->registerPolicy(policy_);
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
                streams_.push_back(new ostreamCamera());
            }else if(std::string(t) == "int"){
                streams_.push_back(new ostreamInt());
            }else if(std::string(t) == "all_policy"){
                policy_ = new PolicyAllRequired();
            }else if(std::string(t) == "odometry"){
                // block
            }else if(std::string(t) == "visualization"){
                // block   
            }
        // }
    }

private:

    std::vector<ostream*> streams_;
    Policy * policy_;
    std::vector<Block*> blocks_;

};


template <>
void Pipeline::builder_expansion<ostream*>(ostream *t) {
    streams_.push_back(t);
}

template <>
void Pipeline::builder_expansion<Block*>(Block *t) {
    blocks_.push_back(t);
}


#endif