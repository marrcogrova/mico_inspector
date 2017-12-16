////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include <rgbd_tools/StereoCameras/StereoCameraRealSense.h>
#include <rgbd_tools/StereoCameras/StereoCameraVirtual.h>

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    inline bool StereoCamera::cloud(pcl::PointCloud<PointType_> &_cloud) {
        // 666 TODO: code implementation to avoid cast in main code and encapsulate behaviour in the library
        // Based on Andrei Alexandrescu  multimethods guide http://www.icodeguru.com/CPP/ModernCppDesign/0201704315_ch11.html
        //if(StereoCameraRealSense * camera = dynamic_cast<StereoCameraRealSense *>(this)){
        //    camera->cloud(_cloud);
        //    return true;
        //}else if(StereoCameraVirtual * camera = dynamic_cast<StereoCameraVirtual *>(this)){
        //    camera->cloud(_cloud);
        //    return true;
        //}else{
            std::cerr << "[STEREOCAMERA] cloud method for custom types not implemented for given point type in used StereoCamera." << std::endl;
            return false;
        //}
    }
}
