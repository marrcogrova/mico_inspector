//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92)
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

        void addWord(std::shared_ptr<Word<PointType_>> &_word);

        void eraseWord(std::shared_ptr<Word<PointType_>> &_word);
        
        /// Update multimatchesInliersCfs when a df becomes a cluster
        void updateMMI(int _dfId, int _cfId);
 
        void updateCovisibility(int _clusterId);

        void updatePose(Eigen::Matrix4f &_pose);

        Eigen::Matrix4f getPose();

        typename pcl::PointCloud<PointType_>::Ptr getCloud();

        typename pcl::PointCloud<PointType_>::Ptr getFeatureCloud();

    public:
        std::string timeStamp = "0.00000";
        /// Members
        int id;
        std::vector<int> frames;
        std::unordered_map<int, std::shared_ptr<Word<PointType_>>> wordsReference;

        std::unordered_map<int, std::shared_ptr<DataFrame<PointType_>>> dataframes;

        Eigen::Vector3f     position;
        Eigen::Quaternionf  orientation;
        Eigen::Matrix4f     pose = Eigen::Matrix4f::Identity();

        std::map<int, std::vector<cv::DMatch>>         multimatchesInliersKfs;
        std::map<int, std::vector<cv::DMatch>>         multimatchesInliersCfs;

        //std::unordered_map<int, double> relations;    wtf

        typename pcl::PointCloud<PointType_>::Ptr cloud;
        typename pcl::PointCloud<PointType_>::Ptr featureCloud;
        
        std::vector<cv::Point2f>  featureProjections;
        cv::Mat                   featureDescriptors;

        
        
        // TODO: Temp g2o
        cv::Mat intrinsic;
        cv::Mat distCoeff;

        cv::Mat left;

        Eigen::Affine3f lastTransformation;

        std::vector<int> covisibility;

        bool optimized = false;        

        #ifdef USE_DBOW2
            DBoW2::BowVector signature;
            DBoW2::FeatureVector featVec;
        #endif

    };

    template<typename PointType_>
    std::ostream& operator<<(std::ostream& os, const ClusterFrames<PointType_>& _cluster){

        for(auto &word: _cluster.wordsReference){
            os << word.first << ",";
            for(auto &frame: _cluster.frames){
                if(std::find(word.second->frames.begin(), word.second->frames.end(), frame->id) != word.second->frames.end()){
                    os << "1,";
                }else{
                    os << "0,";
                }
            }
            os << "," ;
            //Palabras
            os << (*word.second);
            //Palabras
            os << std::endl;
        }
        os << " ,";
        for(auto &frame: _cluster.frames){
            os << frame->wordsReference.size() <<",";
        }
        os << std::endl;
        return os;
    }
}

#include <rgbd_tools/map3d/ClusterFrames.inl>

#endif
