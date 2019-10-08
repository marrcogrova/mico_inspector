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

#include <mico/flow/OutPipe.h>

#include <mico/flow/Policy.h>

namespace mico{
    OutPipe::OutPipe(std::string _tag):tag_(_tag){};

    std::string OutPipe::tag() const {return tag_;};
    
    void OutPipe::registerPolicy(Policy* _pol){
        policiesGuard.lock();
        registeredPolicies_.push_back(_pol);
        policiesGuard.unlock();
    }
    
    void OutPipe::unregisterPolicy(Policy* _pol){
        auto iter = std::find(registeredPolicies_.begin(), registeredPolicies_.end(), _pol);
        if(iter != registeredPolicies_.end()){
            policiesGuard.lock();
            registeredPolicies_.erase(iter);
            policiesGuard.unlock();
        }
    }

    void OutPipe::flush(std::any _data){
        policiesGuard.lock();
        // std::cout << "Flushing " << tag_ << std::endl;
        for(auto &pol: registeredPolicies_){
            pol->update(tag_, _data);
        }
        policiesGuard.unlock();
    }

    int OutPipe::registrations(){
        return registeredPolicies_.size();
    }

}
