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


namespace mico {
    template<typename PointType_>
    inline Entity<PointType_>::Entity(int _id, int _dataframeId, int _label, float _confidence, std::vector<float> _boundingbox,
                                    Eigen::Matrix4f &_pose, typename pcl::PointCloud<PointType_>::Ptr _cloud, 
                                    std::vector<cv::Point2f> _projections, cv::Mat _descriptors){
        id = _id;
        label = _label;
        confidence[_dataframeId] = _confidence;
        boundingbox[_dataframeId] = _boundingbox;
        clouds[_dataframeId] = _cloud;
        projections[_dataframeId] = _projections;
        descriptors[_dataframeId] = _descriptors.clone();
    }

    template<typename PointType_>
    inline int Entity<PointType_>::getId(){
        return id;
    }

    template<typename PointType_>
    inline void Entity<PointType_>::updatePose(Eigen::Matrix4f &_pose){
            pose          = _pose;
            position      = _pose.block<3,1>(0,3);
            orientation   = Eigen::Quaternionf(_pose.block<3,3>(0,0));
    }

    template<typename PointType_>
    inline void Entity<PointType_>::updateCovisibility(int _dataframeId, Eigen::Matrix4f &_pose){
        covisibility[_dataframeId] = _pose;
    }

    template<typename PointType_>
    inline Eigen::Matrix4f Entity<PointType_>::getPose(){
        return pose;
    }

    template<typename PointType_>
    inline typename pcl::PointCloud<PointType_>::Ptr  Entity<PointType_>::getCloud(int _dataframeId){
        return clouds[_dataframeId];
    }
}