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

#ifndef RGBDTOOLS_MAP3D_CLUSTERFRAMES_H_
#define RGBDTOOLS_MAP3D_CLUSTERFRAMES_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <rgbd_tools/map3d/Word.h>
#define USE_DBOW2 1
#ifdef USE_DBOW2
    #include <DBoW2/DBoW2.h>
#endif

#include <functional>
#include <map>
#include <rgbd_tools/map3d/DataFrame.h>

namespace rgbd{
    template<typename PointType_>
    struct ClusterFrames{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// Public constructor
        ClusterFrames(std::shared_ptr<DataFrame<PointType_>> &_df, int _clusterId);

        void addDataframe(std::shared_ptr<DataFrame<PointType_>> &_df);

        /// Members
        int id;
        std::vector<int> frames;
        std::unordered_map<int, double> relations;
        std::unordered_map<int, std::shared_ptr<Word>> wordsReference;

        typename pcl::PointCloud<PointType_>::Ptr cloud;
        typename pcl::PointCloud<PointType_>::Ptr featureCloud;
        std::vector<cv::Point2f>  featureProjections;
        cv::Mat                   featureDescriptors;

        std::map<int, std::vector<cv::DMatch>>  multimatchesInliersClusterFrames;
        
        // TODO: Temp g2o
        cv::Mat intrinsic;
        cv::Mat distCoeff;
        std::map<int,Eigen::Vector3f,std::less<int>,Eigen::aligned_allocator<std::pair<const int, Eigen::Vector3f> >> positions;
        std::map<int,Eigen::Quaternionf,std::less<int>,Eigen::aligned_allocator<std::pair<const int, Eigen::Quaternionf> >>  orientations;
        std::map<int,Eigen::Matrix4f,std::less<int>,Eigen::aligned_allocator<std::pair<const int, Eigen::Matrix4f> >>     poses;

        Eigen::Affine3f lastTransformation;
      
        Eigen::Vector3f     position;
        Eigen::Quaternionf  orientation;
        Eigen::Matrix4f     pose = Eigen::Matrix4f::Identity();

        bool optimized = false;        

        #ifdef USE_DBOW2
            DBoW2::BowVector signature;
            DBoW2::FeatureVector featVec;
        #endif

        friend std::ostream& operator<<(std::ostream& os, const ClusterFrames<PointType_>& _cluster);

    };
}

#include <rgbd_tools/map3d/ClusterFrames.inl>

#endif
