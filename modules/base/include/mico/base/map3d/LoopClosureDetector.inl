
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

#include <map>

namespace mico{
    //---------------------------------------------------------------------------------------------------------------------
    template <DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    template<typename PointType_>
    inline std::vector<std::shared_ptr<Dataframe<PointType_>>> 
        LoopClosureDetector<DebugLevel_, OutInterface_>::findPath(
            std::shared_ptr<Dataframe<PointType_>> _init, 
            std::shared_ptr<Dataframe<PointType_>> _end
            ){

        std::map<int, bool> visitedMap;
        std::vector<std::vector<Dataframe<PointType_>::Ptr>> branches;
        int currentId = df->id;

        // Set init node
        branches.push_back({_init});

        std::vector<Dataframe<PointType_>::Ptr> loopPath;
        bool reachedEnd = false;
        while(!reachedEnd){ // Iterate until reached end node
            for(auto iter = branches.begin(); ++iter; iter != branches.end() && !reachedEnd){    // Iterate over branches
                // Apice of branch
                auto apice = iter->end();
                for(auto &other: _list.back()->covisibility()){ // Iterate over covisibility;
                    if(p == result.matchId){
                        reachedEnd = true;
                        loopPath = *itre;
                        break;
                    }else{
                        if(!visitedMap[other->id()]){
                            iter->push_back(other);
                            visitedMap[other->id()] = true;
                        }
                    }
                    
                }    
                // Remove old branch list
                branches.erase(iter);
            }
        }
    }

}



