//---------------------------------------------------------------------------------------------------------------------
//  mico
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


#ifndef MICO_BASE_MAP3D_DATAFRAME_H_
#define MICO_BASE_MAP3D_DATAFRAME_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <mico/base/map3d/Word.h>
#ifdef USE_DBOW2
    #include <DBoW2/DBoW2.h>
#endif

#include <Eigen/Eigen>

#include <map>

namespace mico {
    template<typename PointType_>
    class Dataframe{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        /// Public type of shared_ptr of Dataframe.
        typedef std::shared_ptr<Dataframe<PointType_>> Ptr;

        /// Dataframes can only be created with ID which uniquely define it.
        Dataframe(int _id);
        
        /// Add a new word to the internal map of words.
        void addWord(std::shared_ptr<Word<PointType_>> &_word);

        /// Delete existing word in the internal map of words.
        void eraseWord(std::shared_ptr<Word<PointType_>> &_word);
        
        /// Update multimatchesInliersCfs when a df becomes a cluster.
        void updateMMI(int _dfId, int _cfId);
 
        /// Add the ID of another DF which is observed from this one
        void appendCovisibility(int _otherId);

        /// Set the pose of the dataframe
        void pose(Eigen::Matrix4f &_pose);

        /// Get current estimated pose of the dataframe
        Eigen::Matrix4f pose() const;

        /// Set dense point cloud of the dataframe
        void cloud(pcl::PointCloud<PointType_>::Ptr &_ could);

        /// Get dense point cloud of the dataframe
        typename pcl::PointCloud<PointType_>::Ptr cloud() const;

        /// Set feature/sparse point cloud of the dataframe
        void featureCloud(pcl::PointCloud<PointType_>::Ptr &_cloud);
        
        /// get feature/sparse point cloud of the dataframe
        typename pcl::PointCloud<PointType_>::Ptr featureCloud() const;

        /// Set feature descriptors associated to the dataframe
        void featureDescriptors(cv::Mat &_descriptors);

        /// Get feature descriptors associated to the dataframe
        cv::Mat featureDescriptors() const;

        /// Set feature projections associated to the dataframe
        void featureProjections(std::vector<cv::Point2f> &_projs);

        /// Get feature projections associated to the dataframe
        cv::Mat featureProjections() const;

        /// Get if df has been optimized.
        bool isOptimized() const;

        /// Set df as optimized
        void isOptimized(bool _opt);

        /// Set image associated to the left camera of the dataframe
        void leftImage(cv::Mat &_image);

        /// Get image associated to the left camera of the dataframe
        cv::Mat leftImage() const;

        /// Get image associated to the right camera of the dataframe
        cv::Mat rightImage() const;

        /// Set image associated to the right camera of the dataframe
        void rightImage(cv::Mat &_image);
        
        /// Get image associated to the depth camera of the dataframe
        cv::Mat depthImage() const;

        /// Set image associated to the depth camera of the dataframe
        void depthImage(cv::Mat &_image);

        /// Get intrinsic coefficients of the camera associated to the dataframe
        cv::Mat intrinsics() const;

        /// Set intrinsic coefficients of the camera associated to the dataframe
        void instrinsics(cv::Mat &_intrinsics);

        /// Get distorsion coefficients of the camera associated to the dataframe
        cv::Mat distCoeff() const;

        /// Set distorsion coefficients of the camera associated to the dataframe
        void distCoeff(cv::Mat &_coeff);

        

    private:
        Dataframe();    // Private void constructor to prevent its creation without ID.

    private:
        // ID of dataframe
        int id_;
        
        // Dense cloud of the dataframe
        typename pcl::PointCloud<PointType_>::Ptr cloud_;
        
        // Feature cloud of the dataframe and related information
        typename pcl::PointCloud<PointType_>::Ptr featureCloud_;
        cv::Mat                         featureDescriptors_;
        std::vector<cv::Point2f>        featureProjections_;

        // Cross reference of features in the other DFs
        std::map<int, std::vector<cv::DMatch>>         multimatchesInliersDfs_;
        std::vector<std::shared_ptr<Word<PointType_>>>          wordsReference_;

        // Reference of which other df is observed from this one.
        std::vector<int> covisibility_;

        // Visual information of the dataframe
        cv::Mat left_, right_, depth_;

        // Pose information 
        Eigen::Vector3f     position_;
        Eigen::Quaternionf  orientation_;
        Eigen::Matrix4f     pose_ = Eigen::Matrix4f::Identity();

        Eigen::Affine3f lastTransformation_;

        // Camera information
        cv::Mat intrinsic_;
        cv::Mat coefficients_;

        // Util flags
        bool optimized_ = false;

        // Signature of dataframe  666 Possible optimization with trait? so can use different implementations
        #ifdef USE_DBOW2
            DBoW2::BowVector signature_;
            DBoW2::FeatureVector featVec_;
        #endif
    };
}

#include <mico/base/map3d/Dataframe.inl>

#endif
