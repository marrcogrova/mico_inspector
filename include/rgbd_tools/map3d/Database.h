////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

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
