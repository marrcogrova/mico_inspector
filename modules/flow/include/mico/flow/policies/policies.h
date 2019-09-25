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
            void setCallback(std::function<void(std::vector<std::any> _data, std::vector<bool> _valid)> _callback);

            virtual bool hasMet();

            int setupStream();

            void update(std::any _val, int _id);
    
        protected:
            std::vector<std::any>   dataFlow_;
            std::vector<bool>       validData_; 
            std::function<void(std::vector<std::any> _data, std::vector<bool> _valid)> callback_;
    };

    class PolicyAllRequired : public Policy{
        public:
        virtual bool hasMet() override;

    };

    class PolicyAny : public Policy{
        public:
        virtual bool hasMet() override;

    };
}



#endif