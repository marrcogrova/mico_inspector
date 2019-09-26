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

#include <mico/flow/policies/policies.h>


namespace mico{

    void Policy::setCallback(std::function<void(std::unordered_map<std::string, std::any> _data, std::unordered_map<std::string,bool> _valid)> _callback){
        callback_ = _callback;
    }

    bool Policy::hasMet(){
        return false;
    };

    void Policy::setupStream(std::string _tag){
        dataFlow_[_tag] = std::any();
        validData_[_tag] = false;
    }

    void Policy::update(std::any _val, std::string _tag){
        dataFlow_[_tag] = _val;
        validData_[_tag] = true;
        if(hasMet()){
            if(callback_)
                callback_(dataFlow_, validData_);
                // std::thread (callback_,dataFlow_, validData_).detach(); // 666 Allow thread detaching and so on...

            for(auto &v: validData_){
                v.second = false;
            }
        }
    }

    bool PolicyAllRequired::hasMet(){
        int counter = 0;
        for(auto v: validData_){
            if(v.second) counter++;
        }
        return counter == validData_.size();
    }


    bool PolicyAny::hasMet(){
        int counter = 0;
        for(auto v: validData_){
            if(v.second) counter++;
        }
        return counter != 0;
    }


}