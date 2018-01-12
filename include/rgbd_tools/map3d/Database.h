////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#ifndef RGBDTOOLS_MAP3D_DATABASE_H_
#define RGBDTOOLS_MAP3D_DATABASE_H_

#include <rgbd_tools/map3d/keyframe.h>
#include <rgbd_tools/map3d/Word.h>

namespace rgbd{

    template<typename PointType_>
    class Database{
    public:
        void addKeyframe(std::shared_ptr<Keyframe<PointType_>> &_kf);
        void connectKeyframes(unsigned _id1, unsigned _id2);

        std::vector<std::shared_ptr<Keyframe<PointType_>>>  keyframes       ()              {return mKeyframes; }
        std::shared_ptr<Keyframe<PointType_>>               keyframe        (unsigned _id)  {return mKeyframes[_id];}
        unsigned                                            numKeyframes    ()              {return mKeyframes.size();}

        std::map<int, std::shared_ptr<Word>> dictionary(){return mWorldDictionary;}
    private:
        std::vector<std::shared_ptr<Keyframe<PointType_>>> mKeyframes;
        std::map<int, std::shared_ptr<Word>> mWorldDictionary;
    };
}

#include "Database.inl"

#endif
