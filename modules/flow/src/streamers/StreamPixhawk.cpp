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


#include <mico/flow/streamers/StreamPixhawk.h>

namespace mico{
        bool StreamPixhawk::configure(std::unordered_map<std::string, std::string> _params) {
            if(run_) // Cant configure if already running.
                return false;
            
            return px_.init(_params);
        }
        
        std::vector<std::string> StreamPixhawk::parameters(){
            return {
                "connection"
            };
        }

        void StreamPixhawk::streamerCallback() {
            while(run_){
                std::this_thread::sleep_for(std::chrono::milliseconds(30)); // 666 Configure it as px freq
                if(registeredPolicies_["acceleration"].size() !=0 ){
                    updatePolicies("acceleration", px_.acceleration());     
                }
                if(registeredPolicies_["orientation"].size() !=0 ){
                    updatePolicies("orientation", px_.orientation());
                }
                if(registeredPolicies_["angular_speed"].size() !=0 ){
                    updatePolicies("angular_speed", px_.angularSpeed());
                }
                if(registeredPolicies_["position"].size() !=0 ){
                    updatePolicies("position", px_.position());
                }
            }      
        }
}