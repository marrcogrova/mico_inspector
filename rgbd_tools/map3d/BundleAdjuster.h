////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#ifndef RGBDTOOLS_MAP3D_BUNDLEADJUSTER_H_
#define RGBDTOOLS_MAP3D_BUNDLEADJUSTER_H_

#include <map3d/keyframe.h>

namespace rgbd{
    template<typename PointType_>
    class BundleAdjuster{
    public:
        bool optimize();
        void keyframes(std::vector<Keyframe<PointType_>> &_keyframes);
        void keyframes(std::vector<Keyframe<PointType_>>::iterator &_begin, std::vector<Keyframe<PointType_>>::iterator &_end);

    private:
        bool prepareData();

    private:
        std::vector<Keyframe<PointType_>> mKeyframes;

    };
}   // namespace rgbd

#include <map3d/BundleAdjuster.inl>

#endif //RGBDTOOLS_MAP3D_BUNDLEADJUSTER_H_
