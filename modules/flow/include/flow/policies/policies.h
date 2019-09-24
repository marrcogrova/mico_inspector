

#ifndef MICO_FLOW_POLICIES_POLICIES_H_
#define MICO_FLOW_POLICIES_POLICIES_H_

#include <vector>
#include <cstdlib>

#include <any>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <iostream>
#include <functional>

#include <opencv2/opencv.hpp>

namespace mico{
    
    class Policy{
    public:
            void setCallback(std::function<void(std::vector<std::any> _data)> _callback){
                callback_ = _callback;
            }

            virtual bool hasMet(){
                return false;
            };

            int setupStream(){
                dataFlow_.push_back(std::any());
                validData_.push_back(false);
                return dataFlow_.size()-1;
            }

            void update(std::any _val, int _id){
                dataFlow_[_id] = _val;
                validData_[_id] = true;
                if(hasMet()){
                    if(callback_)
                        callback_(dataFlow_);
                        // std::thread (callback_,dataFlow_).detach(); // 666 Allow thread detaching and so on...

                    for(int i = 0; i < validData_.size(); i++){
                        validData_[i] = false;
                    }
                }
            }

            std::vector<std::any>   dataFlow_;
            std::vector<bool>       validData_; 
            std::function<void(std::vector<std::any> _data)> callback_;
    };

    class PolicyAllRequired : public Policy{
    public:
        virtual bool hasMet() override{
            int counter = 0;
            for(auto v: validData_){
                if(v) counter++;
            }
            return counter == validData_.size();
        }

    };
}



#endif