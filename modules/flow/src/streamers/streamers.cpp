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

    Ostream::Ostream(int _nStreams, std::vector<std::string> _streamTags):  nStreams_(_nStreams), 
                                                                            streamTags_(_streamTags),
                                                                            registeredPolicies_(_nStreams){

    }
        

    void Ostream::registerPolicy(Policy *_policy, int _stream){
        assert(_stream < nStreams_);
        int internalId = _policy->setupStream();
        registeredPolicies_[_stream][_policy] = internalId;
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

    void Ostream::updatePolicies(int _stream, std::any _data){
        for(auto &pol : registeredPolicies_[_stream]){
            pol.first->update(_data, pol.second);
        }
    }

    void OstreamCamera::streamerCallback(){
        camera_ = new cv::VideoCapture(0);
        while(run_){
            cv::Mat image, gray;
            camera_->grab();
            *camera_ >> image;
            std::this_thread::sleep_for(std::chrono::milliseconds((int) 0.5*1000));
            updatePolicies(0,image);
            cv::cvtColor(image, gray, cv::ColorConversionCodes::COLOR_BGR2GRAY);
            updatePolicies(1,gray);
        }
        camera_->release();
    }



}
