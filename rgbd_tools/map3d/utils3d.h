////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#ifndef RGBDTOOLS_MAP3D_UTILS3D_H_
#define RGBDTOOLS_MAP3D_UTILS3D_H_

#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace rgbd{

    template<typename PointType_>
    void ransacAlignment(typename pcl::PointCloud<PointType_>::Ptr _source,
                         typename pcl::PointCloud<PointType_>::Ptr _target,
                         std::vector<cv::DMatch> &_matches,
                         Eigen::Matrix4f &_transformation,
                         std::vector<int> &_inliers,
                         double _maxRansacDistance = 0.01,
                         int _ransacIterations = 3000,
                         unsigned _refineIterations = 5);


}

#include "utils3d.inl"

#endif
