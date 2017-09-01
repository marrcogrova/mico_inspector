////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#ifndef RGBDSLAM_MAP3D_KEYFRAME_H_
#define RGBDSLAM_MAP3D_KEYFRAME_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include "Word.h"

namespace rgbd{
    template<typename PointType_>
    struct Keyframe{
        int mId;
        typename pcl::PointCloud<PointType_>::Ptr cloud;
        typename pcl::PointCloud<PointType_>::Ptr featureCloud;
        std::vector<cv::Point2f>        featureProjections;
        cv::Mat                         featureDescriptors;
        std::vector<cv::DMatch>         matchesPrev;
        std::vector<cv::DMatch>         ransacInliers;
        std::vector<&Word>              wordsReference;

        Eigen::Vector3f     position;
        Eigen::Quaternionf  orientation;
        Eigen::Matrix4f     pose = Eigen::Matrix4f::Identity();

        cv::Mat intrinsic;
        cv::Mat coefficients;

        // 777 for debugging
        cv::Mat left, right, depth;
    };
}

#endif
