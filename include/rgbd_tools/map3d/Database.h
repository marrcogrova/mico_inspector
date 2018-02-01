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


#ifndef RGBDTOOLS_MAP3D_DATABASE_H_
#define RGBDTOOLS_MAP3D_DATABASE_H_

#include <rgbd_tools/map3d/DataFrame.h>
#include <rgbd_tools/map3d/Word.h>

namespace rgbd{

    template<typename PointType_>
    class Database{
    public:
        void addKeyframe(std::shared_ptr<DataFrame<PointType_>> &_kf);
        void connectKeyframes(unsigned _id1, unsigned _id2, bool _has3D = true);
        void reset();

        std::vector<std::shared_ptr<DataFrame<PointType_>>>  keyframes       ()              {return mKeyframes; }
        std::shared_ptr<DataFrame<PointType_>>               keyframe        (unsigned _id)  {return mKeyframes[_id];}
        unsigned                                            numKeyframes    ()              {return mKeyframes.size();}

        std::map<int, std::shared_ptr<Word>> dictionary(){return mWorldDictionary;}
        pcl::PointCloud<PointType_> wordMap(){return mWordMap;}
    private:
        std::vector<std::shared_ptr<DataFrame<PointType_>>> mKeyframes;
        std::map<int, std::shared_ptr<Word>> mWorldDictionary;
        pcl::PointCloud<PointType_> mWordMap;

    private: // helper functions
        Eigen::Vector3f triangulateFromProjections(std::unordered_map<int, std::vector<float>>);
    };
}

#include "Database.inl"

#endif
