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

#ifndef MICO_DNN_MAP3D_ENTITY_H_
#define MICO_DNN_MAP3D_ENTITY_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

namespace mico
{

template <typename PointType_>
struct Entity
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    typedef std::shared_ptr<Entity<PointType_>> Ptr;

    Entity(int _id, int _dataframeId, int _label, float _confidence, std::vector<float> _boundingbox);
    Entity(int _id, int _label, float _confidence, std::vector<float> _boundingbox);

    // getters
    int getId();
    Eigen::Matrix4f getPose();
    typename pcl::PointCloud<PointType_>::Ptr getCloud(int _id);

    /// setters
    void setPose(Eigen::Matrix4f &_pose);
    void setCloud(int _dataframeId, typename pcl::PointCloud<PointType_>::Ptr &_cloud);
    void setProjections(int _dataframeId, std::vector<cv::Point2f> _projections);
    void setDescriptors(int _dataframeId, cv::Mat _descriptors);
    void updateCovisibility(int _dataframeId, Eigen::Matrix4f &_pose);
    
private:
    Entity(){};

    int id;

    /// spatial data 
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    
    /// 3D
    std::map<int, typename pcl::PointCloud<PointType_>::Ptr> clouds;

    /// 2D
    std::map<int, std::vector<cv::Point2f>> projections;
    std::map<int, cv::Mat> descriptors;

    /// detection 
    int label;    
    std::map<int, float> confidence;
    std::map<int, std::vector<float>> boundingbox;     // left top right bottom
    std::map<int, std::vector<float>> boundingcube;    // xmax xmin ymax ymin zmax zmin
    
    /// visibility
    std::map<int,Eigen::Matrix4f> covisibility;

};
} // namespace mico
#include <mico/dnn/map3d/Entity.inl>

#endif