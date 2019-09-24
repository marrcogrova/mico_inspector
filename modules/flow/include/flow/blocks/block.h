

#ifndef MICO_FLOW_STREAMERS_BLOCKS_BLOCK_H_
#define MICO_FLOW_STREAMERS_BLOCKS_BLOCK_H_

#include <mico/flow/streamers.h>
#include <vector>
#include <functional>

class Block{
public:
    void registerCallback(std::function<void(std::vector<std::any> _data)> _callback){
        callback_ = _callback;
    }
    
    void setPolicy(Policy*_pol){
        iPolicy_ = _pol;
        iPolicy_->setCallback([&](std::vector<std::any> _data){this->callback_(_data);});
    }

    void operator()(std::vector<std::any> _data){
        callback_(_data);
    }

    Policy *iPolicy_;
    std::vector<ostream> ostreams_;
    std::function<void(std::vector<std::any> _data)> callback_;
};


#endif