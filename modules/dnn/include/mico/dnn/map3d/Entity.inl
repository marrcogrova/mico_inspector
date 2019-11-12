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
    inline Entity<PointType_>::Entity(int _id, int _dataframeId, int _label, float _confidence, std::vector<float> _boundingbox){
        id_ = _id;
        label_ = _label;
        confidence_[_dataframeId] = _confidence;
        boundingbox_[_dataframeId] = _boundingbox;
        computePCA(_dataframeId);
    }

    template<typename PointType_>
    inline Entity<PointType_>::Entity(int _id, int _label, float _confidence, std::vector<float> _boundingbox):
    Entity(_id, 0,_label, _confidence,  _boundingbox){
    }

    template<typename PointType_>
    inline bool Entity<PointType_>::computePCA(int _dataframeId){   // 666 _dataframeId to compute pose with several clouds        
        // Compute principal directions
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid<pcl::PointXYZRGBNormal>(*clouds_[_dataframeId], pcaCentroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized<pcl::PointXYZRGBNormal>(*clouds_[_dataframeId], pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                       ///    the signs are different and the box doesn't get correctly oriented in some cases.
        
        //// PCA comparison
        //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        //pcl::PCA<pcl::PointXYZRGBNormal> pca;
        //pca.setInputCloud(clouds[_dataframeId]);
        ////pca.project(*cloud, *cloudPCAprojection);
        //std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
        //std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
        //// In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.

        // Transform the original cloud to the origin where the principal components correspond to the axes.
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::transformPointCloud(*clouds_[_dataframeId], *cloudPointsProjected, projectionTransform);

        // Get the minimum and maximum points of the transformed cloud.
        pcl::PointXYZRGBNormal minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        boundingcube_[_dataframeId] = {maxPoint.x, minPoint.x, maxPoint.y, minPoint.y, maxPoint.z, minPoint.z};

        const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

        // Final transform
        const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
        const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block(0,0,3,3) = bboxQuaternion.toRotationMatrix();
        pose.block(0,3,3,1) = bboxTransform;

        poses_[_dataframeId] = pose * covisibility_[_dataframeId];   // to global pose 666 check this
        positions_[_dataframeId] = bboxTransform;
        orientations_[_dataframeId] = bboxQuaternion;
        // visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox1", mesh_vp_2);
    }

    template<typename PointType_>
    inline void Entity<PointType_>::updateCovisibility(int _dataframeId, Eigen::Matrix4f &_pose){
        covisibility_[_dataframeId] = _pose;
    }

    template<typename PointType_>
    inline void Entity<PointType_>::pose(int _dataframeId, Eigen::Matrix4f &_pose){
            poses_[_dataframeId]   = _pose;
            positions_[_dataframeId]   = _pose.block<3,1>(0,3);
            orientations_[_dataframeId]   = Eigen::Quaternionf(_pose.block<3,3>(0,0));
    }

    template<typename PointType_>
    inline void Entity<PointType_>::cloud(int _dataframeId, typename pcl::PointCloud<PointType_>::Ptr &_cloud){
            clouds_[_dataframeId] = _cloud;
    }

    template<typename PointType_>
    inline void Entity<PointType_>::projections(int _dataframeId, std::vector<cv::Point2f> _projections){
            projections_[_dataframeId] = _projections;
    }

    template<typename PointType_>
    inline void Entity<PointType_>::descriptors(int _dataframeId, cv::Mat _descriptors){
            descriptors_[_dataframeId] = _descriptors.clone();
    }
    
    template<typename PointType_>
    inline int Entity<PointType_>::id() const{
        return id_;
    }

    template<typename PointType_>
    inline Eigen::Matrix4f Entity<PointType_>::pose(int _dataframeId){
        return poses_[_dataframeId];
    }

    template<typename PointType_>
    inline typename pcl::PointCloud<PointType_>::Ptr  Entity<PointType_>::cloud(int _dataframeId) const{
        return clouds_[_dataframeId];
    }
}