//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
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

#include <rgbd_tools/map3d/ClusterFrames.h>

#include <iostream>

namespace rgbd{
template<typename PointType_>
std::ostream& operator<<(std::ostream& os, const ClusterFrames<PointType_>& _cluster){

    for(auto &word: _cluster.wordsReference){
        os << word.first << ",";
        for(auto &frame: _cluster.frames){
            if(std::find(word.second->frames.begin(), word.second->frames.end(), frame->id) != word.second->frames.end()){
                os << "1,";
            }else{
                os << "0,";
            }
        }
        os << "," ;
        //Palabras
        os << (*word.second);
        //Palabras
        os << std::endl;
    }
    os << " ,";
    for(auto &frame: _cluster.frames){
        os << frame->wordsReference.size() <<",";
    }
    os << std::endl;
    return os;
    }
}

