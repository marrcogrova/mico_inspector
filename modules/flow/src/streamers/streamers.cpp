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

namespace mico{

    void ostream::registerPolicy(Policy *_policy){
        int internalId = _policy->setupStream();
        registeredPolicies_[_policy] = internalId;
    }

    void ostream::start(){
        run_ = true;
        loop_ = std::thread(&ostream::streamerCallback, this);
    }

    void ostream::stop(){
        run_ = false;
        if(loop_.joinable())
            loop_.join();
    }

    void ostream::updatePolicies(std::any _data){
        for(auto &pol : registeredPolicies_){
            pol.first->update(_data, pol.second);
        }
    }

    void ostreamCamera::streamerCallback(){
        camera_ = new cv::VideoCapture(0);
        while(run_){
            cv::Mat image;
            camera_->grab();
            *camera_ >> image;
            std::any val =  image;
            std::this_thread::sleep_for(std::chrono::milliseconds((int) 0.5*1000));
            updatePolicies(val);
        }
        camera_->release();
    }



}
