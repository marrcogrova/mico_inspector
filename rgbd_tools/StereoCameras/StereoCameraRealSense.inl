////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////


#include <pcl/features/integral_image_normal.h>

namespace rgbd{
    //---------------------------------------------------------------------------------------------------------------------
    template<typename PointType_>
    bool StereoCameraRealSense::cloud(pcl::PointCloud<PointType_> &_cloud) {
        if (!mUseUncolorizedPoints) {
            std::cout << "[STEREOCAMERA][REALSENSE] Cannot compute the normals if points out of the colorized region are ignored. Please set the \"UseUncolorizedPoints\" to true in the configuration of the camera" << std::endl;
            return false;
        }
        else {
            pcl::PointCloud<pcl::PointXYZRGB> cloudWoNormals;
            if (!cloud(cloudWoNormals)) {
                return false;
            }

            if(cloudWoNormals.size() == 0){
                std::cout << "[STEREOCAMERA][REALSENSE] Empty cloud, can't compute normals" << std::endl;
                _cloud.resize(0);
                return true;
            }

            //pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
            pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, PointType_> ne;
            ne.setInputCloud(cloudWoNormals.makeShared());
            ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
            ne.setMaxDepthChangeFactor(0.02f);
            ne.setNormalSmoothingSize(10.0f);
            ne.compute(_cloud);

            // Fill XYZ and RGB of cloud
            for (unsigned i = 0; i < _cloud.size(); i++) {
                _cloud[i].x = cloudWoNormals[i].x;
                _cloud[i].y = cloudWoNormals[i].y;
                _cloud[i].z = cloudWoNormals[i].z;
                _cloud[i].r = cloudWoNormals[i].r;
                _cloud[i].g = cloudWoNormals[i].g;
                _cloud[i].b = cloudWoNormals[i].b;
            }

            return true;
        }
    }
}
