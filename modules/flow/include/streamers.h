


#ifndef STREAMERS_H_
#define STREAMERS_H_

#include <vector>
#include <cstdlib>

#include <any>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <iostream>
#include <functional>

#include <opencv2/opencv.hpp>


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


class ostream{
public:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void updatePolicies() = 0;

    void registerPolicy(Policy *_policy){
        int internalId = _policy->setupStream();
        registeredPolicies_[_policy] = internalId;
    }

    std::unordered_map<Policy*, int> registeredPolicies_;
};

class ostreamInt:public ostream{
public:
    virtual void start(){
        sleepTime_ = 0.1;
        run_ = true;
        loop_ = std::thread([&](){
            while(run_){
                std::this_thread::sleep_for(std::chrono::milliseconds((int) sleepTime_*1000));
                updatePolicies();
            }
        });
    }

    virtual void stop(){
        run_ = false;
        if(loop_.joinable())
            loop_.join();
    }

    virtual void updatePolicies() override{
        std::any val = (int) std::rand();
        for(auto &pol : registeredPolicies_){
            pol.first->update(val, pol.second);
        }
    }

    std::thread loop_;
    float sleepTime_ = 0;
    bool run_ = false;
};


class ostreamString:public ostream{
public:
    virtual void start(){
        sleepTime_ = 0.1;
        run_ = true;
        loop_ = std::thread([&](){
            while(run_){
                std::this_thread::sleep_for(std::chrono::milliseconds((int) sleepTime_*1000));
                updatePolicies();
            }
        });
    }

    virtual void stop(){
        run_ = false;
        if(loop_.joinable())
            loop_.join();
    }

    virtual void updatePolicies() override{
        std::string raw = "";
        int nChars = float(std::rand())/RAND_MAX*15+5;
        for(int i = 0; i < nChars; i++){
            raw += 'a'+int(float(std::rand())/RAND_MAX*30);
        }
        std::any val =  raw;
        for(auto &pol : registeredPolicies_){
            pol.first->update(val, pol.second);
        }
    }

    std::thread loop_;
    float sleepTime_ = 0;
    bool run_ = false;
};



class ostreamCamera:public ostream{
public:
    virtual void start(){
        sleepTime_ = 0.5;
        run_ = true;
        loop_ = std::thread([&](){
            camera_ = new cv::VideoCapture(0);
            while(run_){
                std::this_thread::sleep_for(std::chrono::milliseconds((int) sleepTime_*1000));
                updatePolicies();
            }
        });
    }

    virtual void stop(){
        run_ = false;
        if(loop_.joinable())
            loop_.join();
        camera_->release();
    }

    virtual void updatePolicies() override{
        cv::Mat image;
        camera_->grab();
        *camera_ >> image;
        std::any val =  image;
        for(auto &pol : registeredPolicies_){
            pol.first->update(val, pol.second);
        }
    }

    cv::VideoCapture *camera_ ;
    std::thread loop_;
    float sleepTime_ = 0;
    bool run_ = false;
};



#endif