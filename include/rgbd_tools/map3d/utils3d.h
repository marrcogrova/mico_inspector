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

#ifndef RGBDTOOLS_MAP3D_UTILS3D_H_
#define RGBDTOOLS_MAP3D_UTILS3D_H_

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include <rgbd_tools/map3d/utils2d.h>
#include <rgbd_tools/map3d/DataFrame.h>
#include <rgbd_tools/map3d/ClusterFrames.h>

#include <rgbd_tools/utils/LogManager.h>

namespace rgbd{

    /// Perform alignment between two clouds using RANSAC
    /// \param _source:
    /// \param _target:
    /// \param _matches:
    /// \param _transformation:
    /// \param _inliers:
    /// \param _maxRansacDistance:
    /// \param _ransacIterations:
    /// \param _refineIterations:
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Cout>
    void ransacAlignment(typename pcl::PointCloud<PointType_>::Ptr _source,
                         typename pcl::PointCloud<PointType_>::Ptr _target,
                         std::vector<cv::DMatch> &_matches,
                         Eigen::Matrix4f &_transformation,
                         std::vector<int> &_inliers,
                         double _maxRansacDistance = 0.01,
                         int _ransacIterations = 3000,
                         unsigned _refineIterations = 5);

    /// Perform alignement between two clouds given an initial transformation
    /// \param _source:
    /// \param _target:
    /// \param _transformation:
    /// \param _correspondenceDistance:
    /// \param _maxAngleDistance:
    /// \param _maxColorDistance:
    /// \param _maxTranslation:
    /// \param _maxRotation:
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Cout>
    bool icpAlignment(typename pcl::PointCloud<PointType_>::Ptr _source,
                      typename pcl::PointCloud<PointType_>::Ptr _target,
                      Eigen::Matrix4f &_transformation,
                      int _iterations = 10,
                      double _correspondenceDistance = 0.3,
                      double _maxAngleDistance = 0.707,
                      double _maxColorDistance = 0.3,
                      double _maxTranslation = 0.01,
                      double _maxRotation = 0.01,
                      double _maxFitnessScore = 1.0
                      );
    /// Compute roughtly but robustly the transformation between given keyframes.
    /// \param _previousKf:
    /// \param _currentKf:
    /// \param _transformation:
    /// \param _mk_nearest_neighbors:
    /// \param _mRansacMaxDistance:
    /// \param _mRansacIterations:
    /// \param _mRansacMinInliers:
    /// \param _mRansacMinInliers:
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Cout>
    bool transformationBetweenFeatures(std::shared_ptr<DataFrame<PointType_>> &_previousKf,
                                       std::shared_ptr<DataFrame<PointType_>> &_currentKf,
                                       Eigen::Matrix4f &_transformation,
                                       double _mk_nearest_neighbors,
                                       double _mRansacMaxDistance,
                                       int _mRansacIterations,
                                       double _mRansacMinInliers,
                                       double _mFactorDescriptorDistance);

    /// Compute roughtly but robustly the transformation between current keyframes and cluster.
    /// \param _lastCluster:
    /// \param _currentKf:
    /// \param _transformation:
    /// \param _mk_nearest_neighbors:
    /// \param _mRansacMaxDistance:
    /// \param _mRansacIterations:
    /// \param _mRansacMinInliers:
    /// \param _mRansacMinInliers:
    template<typename PointType_, DebugLevels DebugLevel_ = DebugLevels::Null, OutInterfaces OutInterface_ = OutInterfaces::Cout>
    bool transformationBetweenwordsReference(std::shared_ptr<ClusterFrames<PointType_>> &_lastCluster,
                                           std::shared_ptr<DataFrame<PointType_>> &_currentKf,
                                           Eigen::Matrix4f &_transformation,
                                           double _mk_nearest_neighbors,
                                           double _mRansacMaxDistance,
                                           int _mRansacIterations,
                                           double _mRansacMinInliers,
                                           double _mFactorDescriptorDistance);

}

#include "utils3d.inl"

#endif
