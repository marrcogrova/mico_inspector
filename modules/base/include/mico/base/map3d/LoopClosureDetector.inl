
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

        // std::map<int, bool> visitedMap;
        // std::vector<std::vector<Dataframe<PointType_>::Ptr>> traces;
        // int currentId = df->id;

        // std::function<void(Dataframe<PointType_>::Ptr&)> expandNode; 
        // bool finished = false;
        // expandNode = [&](   std::vector<typename Dataframe<PointType_>::Ptr> _list)
        //                             ->std::vector<typename Dataframe<PointType_>::Ptr>{
            
        //     if(finished)
        //         return {};

        //     // Remove all nodes used in this loop
        //     std::vector<Dataframe<PointType_>::Ptr> pending;
        //     for(auto &other: _list.back()->covisibility()){
        //         if(!visitedMap[other->id()]){
        //             pending.push_back(other);
        //             visitedMap[other->id()] = true;
        //         }
        //     }

        //     if(pending.size() == 0)
        //         return {};

        //     // iterate over pending options
        //     for(auto &p: pending){
        //         if(p == result.matchId){
        //             finished = true;
        //             return _list;
        //         }else{
        //             auto newList = _list;
        //             newList.push_back(p);
        //             return expandNode(newList);
        //         }
        //     }
        // }

        
    }

}



