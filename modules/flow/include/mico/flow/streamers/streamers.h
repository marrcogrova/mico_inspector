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

namespace mico{

    struct Packet{
        time_t timeStamp_;
        std::any data_;
    };

    // Forward declaration
    class Policy;

    class Ostream{
    public:
        static std::string name() {return "Unnammed";}

        Ostream(std::vector<std::string> _streamTags);

        virtual void configure(std::unordered_map<std::string, std::string> _params) {};
        virtual std::vector<std::string> parameters(){ return {}; };
        
        void manualUpdate(std::unordered_map<std::string, std::any> _data);

        void start();
        void stop();
        void updatePolicies(std::string _tag, std::any _data);

        void registerPolicy(Policy *_policy, std::string _tag);
        void unregisterPolicy(Policy *_policy, std::string _tag);

        int nOutputs();
        std::vector<std::string> outputTags();

    protected:
        virtual void streamerCallback() = 0;

        bool run_ = false;
        std::unordered_map<std::string, std::vector<Policy*>> registeredPolicies_;   // Policy registered, ID of stream and index in policy;
    private:
        std::thread loop_;

        int nStreams_ = 0;
        std::vector<std::string> streamTags_ = {};
    };


    class OstreamCamera:public Ostream{
    public:
        OstreamCamera(): Ostream({"color, gray"}) { }

        virtual void streamerCallback() override;

    private:
        cv::VideoCapture *camera_ ;
    };

}



#endif