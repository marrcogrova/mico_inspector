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


#include <mico/flow/streamers/StreamRealSense.h>

namespace mico{
        void OstreamRealsense::configure(std::unordered_map<std::string, std::string> _params) {
            if(run_) // Cant configure if already running.
                return;

            cjson::Json jParams;
            for(auto &p:_params){
                if(p.first == "deviceId"){
                    jParams["deviceId"] = atoi(p.second.c_str());
                }else if(p.first == "cloudDownsampleStep"){
                    jParams["cloudDownsampleStep"] = atoi(p.second.c_str());
                }else if(p.first == "useUncolorizedPoints"){
                    jParams["useUncolorizedPoints"] = p.second == "true" ? true : false;
                }
            }
            camera_.init(jParams);
        }

        void OstreamRealsense::streamerCallback() {
            
        }
}