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

#include <mico/flow/Policy.h>


namespace mico{

    Policy::Policy(std::vector<std::string> _inPipes){
        tags_ = _inPipes;
        for(auto &tag: _inPipes){
            dataFlow_[tag] = std::any();
            validData_[tag] = false;
        }
    }

    void Policy::setCallback(PolicyMask _mask, PolicyCallback _callback){
        callbacks_.push_back({_mask, _callback});
    }

    void Policy::update(std::string _tag, std::any _val){
        // std::cout << "updated " << _tag << std::endl;
        dataFlow_[_tag] = _val;
        validData_[_tag] = true;
        checkMasks();
    }

    int Policy::nInputs(){
        return tags_.size();
    }

    std::vector<std::string> Policy::inputTags(){
        return tags_;
    }


    void Policy::checkMasks(){
        for(auto &pairCb: callbacks_){  // Check all pairs mask-cb
            auto maskTags = pairCb.first;
            unsigned counter = 0;
            for(auto &tag: maskTags){   // Check all tags in mask
                for(auto iter = validData_.begin(); iter != validData_.end(); iter++){
                    if(iter->first == tag && validData_[tag]){
                        counter++;
                        break;
                    }
                }
            }
            // std::cout << counter << "/" << maskTags.size()  << std::endl;
            if(counter ==  maskTags.size()){
                for(auto&tag:maskTags){ // uff... For more complex pipelines with shared data might not work... need conditions per callback.
                    validData_[tag] = false;
                    // std::cout << "Demarked " << tag << std::endl;
                }
                // std::thread(pairCb.second, dataFlow_).detach();
                pairCb.second(dataFlow_);
            }
        }
    }

}