


#ifndef MICO_FLOW_STREAMERS_STREAMERS_STREAMERS_H_
#define MICO_FLOW_STREAMERS_STREAMERS_STREAMERS_H_

#include <vector>
#include <cstdlib>

#include <any>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <iostream>
#include <functional>

#include <opencv2/opencv.hpp>



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