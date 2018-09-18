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
#ifdef USE_DBOW2
    #include <DBoW2/DBoW2.h>
#endif

#include <functional>
#include <map>
#include <rgbd_tools/map3d/DataFrame.h>
#include <rgbd_tools/map3d/Word.h>

namespace rgbd{
    template<typename PointType_>
    struct ClusterFrames{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        typedef std::shared_ptr<ClusterFrames<PointType_>> Ptr;

        /// Public constructor
        ClusterFrames(std::shared_ptr<DataFrame<PointType_>> &_df, int _clusterId);

        void addDataframe(std::shared_ptr<DataFrame<PointType_>> &_df);

        Eigen::Matrix4f bestPose();

        typename pcl::PointCloud<PointType_>::Ptr bestCloud();

        typename pcl::PointCloud<PointType_>::Ptr bestFeatureCloud();

        std::shared_ptr<DataFrame<PointType_>> bestDataframePtr();

        void switchBestDataframe();

    public:
        /// Members
        int id;
        std::vector<int> frames;
        std::unordered_map<int, std::shared_ptr<Word<PointType_>>> wordsReference;

        std::unordered_map<int, std::shared_ptr<DataFrame<PointType_>>> dataframes;
        int bestDataframe = 0;
        //std::unordered_map<int, double> relations;    wtf

        std::vector<cv::Point2f>  featureProjections;
        cv::Mat                   featureDescriptors;

        std::map<int, std::vector<cv::DMatch>>  multimatchesInliersClusterFrames;
        
        // TODO: Temp g2o
        cv::Mat intrinsic;
        cv::Mat distCoeff;

        Eigen::Affine3f lastTransformation;

        std::vector<int> covisibility;

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
