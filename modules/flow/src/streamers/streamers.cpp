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


#include <mico/flow/streamers/streamers.h>

#include <mico/flow/policies/policies.h>

#include <cassert>

namespace mico{

    Ostream::Ostream(std::vector<std::string> _streamTags): nStreams_(_streamTags.size()), 
                                                            streamTags_(_streamTags),
                                                            registeredPolicies_(_streamTags.size()){

    }
        

    void Ostream::registerPolicy(Policy *_policy, std::string _tag){
        _policy->setupStream(_tag);
        registeredPolicies_[_tag].push_back(_policy);
    }


    int Ostream::nOutputs(){
        return streamTags_.size();
    }
    
    std::vector<std::string> Ostream::outputTags(){
        return streamTags_;
    }

    void Ostream::manualUpdate(std::unordered_map<std::string, std::any> _data){
        for (auto &pair: _data){
            updatePolicies(pair.first, pair.second);
        }
    }

    void Ostream::start(){
        run_ = true;
        loop_ = std::thread(&Ostream::streamerCallback, this);
    }

    void Ostream::stop(){
        run_ = false;
        if(loop_.joinable())
            loop_.join();
    }

    void Ostream::updatePolicies(std::string _tag, std::any _data){
        for(auto &pol : registeredPolicies_[_tag]){
            pol->update(_data, _tag);
        }
    }

    void OstreamCamera::streamerCallback(){
        camera_ = new cv::VideoCapture(0);
        while(run_){
            cv::Mat image, gray;
            camera_->grab();
            *camera_ >> image;
            std::this_thread::sleep_for(std::chrono::milliseconds((int) 0.5*1000));
            updatePolicies("rgb",image);
            cv::cvtColor(image, gray, cv::ColorConversionCodes::COLOR_BGR2GRAY);
            updatePolicies("gray",gray);
        }
        camera_->release();
    }



}
