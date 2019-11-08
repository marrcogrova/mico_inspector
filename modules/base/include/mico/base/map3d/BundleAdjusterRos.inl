//---------------------------------------------------------------------------------------------------------------------
//  mico
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


#ifdef RGBDTOOLS_USE_ROS_DEPRECATED

    #include <mico/base/map3d/cvsba/cvsba.h>
    #include <unordered_map>
    #include <mico/base/utils/Gui.h>
    #include <pcl/common/transforms.h>
    #include <algorithm>
    #include <iterator>

    #include <visualization_msgs/Marker.h> 

    namespace mico {
        //---------------------------------------------------------------------------------------------------------------------
        template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
        inline BundleAdjusterRos<PointType_, DebugLevel_, OutInterface_>::BundleAdjusterRos() {
            ros::NodeHandle nh;
            mCameraMarkerPub = nh.advertise<visualization_msgs::Marker>("/sba/cameras", 1);
            mPointMarkerPub = nh.advertise<visualization_msgs::Marker>("/sba/points", 1);
            ros::spinOnce();
        }

        //---------------------------------------------------------------------------------------------------------------------
        template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
        inline void BundleAdjusterRos<PointType_, DebugLevel_, OutInterface_>::cleanData() {
            if(mSba == nullptr)
                delete mSba;

        mSba = new sba::SysSBA();
        }

        //---------------------------------------------------------------------------------------------------------------------
        template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
        inline void BundleAdjusterRos<PointType_, DebugLevel_, OutInterface_>::reserveData(int _cameras, int _words){
        
        }

        //---------------------------------------------------------------------------------------------------------------------
        template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
        inline void BundleAdjusterRos<PointType_, DebugLevel_, OutInterface_>::appendCamera(int _id, Eigen::Matrix4f _pose, cv::Mat _intrinsics, cv::Mat _distcoeff){
            Vector4d trans(_pose(0,3), _pose(1,3), _pose(2,3), 1.0);
            Quaterniond qrot(_pose.block<3,3>(0,0).cast<double>());
            
            frame_common::CamParams cam_params;
            cam_params.fx = _intrinsics.at<double>(0,0);
            cam_params.fy = _intrinsics.at<double>(1,1);
            cam_params.cx = _intrinsics.at<double>(0,2);
            cam_params.cy = _intrinsics.at<double>(1,2);
            cam_params.tx = 0;
            
            bool fixed = _id == 0;
            
            mSba->addNode(trans, qrot, cam_params, fixed);
            mSba->nodes[_id].normRot();
            mSba->nodes[_id].setTransform();
            mSba->nodes[_id].setProjection();
            mSba->nodes[_id].setDr(true);
        }

        //---------------------------------------------------------------------------------------------------------------------
        template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
        inline void BundleAdjusterRos<PointType_, DebugLevel_, OutInterface_>::appendPoint(int _id, Eigen::Vector3f _position){
            Vector4d point(_position[0], _position[1], _position[2],1);
            mSba->addPoint(point);
        }

        //---------------------------------------------------------------------------------------------------------------------
        template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
        inline void BundleAdjusterRos<PointType_, DebugLevel_, OutInterface_>::appendProjection(int _idCamera, int _idPoint, cv::Point2f _projection){
            Vector2d proj(_projection.x, _projection.y);
            mSba->addMonoProj(_idCamera, _idPoint, proj);
        }

        //---------------------------------------------------------------------------------------------------------------------
        template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
        inline void BundleAdjusterRos<PointType_, DebugLevel_, OutInterface_>::fitSize(int _cameras, int _words){
        
        }

        //---------------------------------------------------------------------------------------------------------------------
        template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
        inline void BundleAdjusterRos<PointType_, DebugLevel_, OutInterface_>::checkData(){
            
        }

        //---------------------------------------------------------------------------------------------------------------------
        template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
        inline bool BundleAdjusterRos<PointType_, DebugLevel_, OutInterface_>::doOptimize(){
            drawGraph(*mSba, mCameraMarkerPub, mPointMarkerPub);
            ros::spinOnce();
            mSba->doSBA(10, 1e-3, SBA_SPARSE_CHOLESKY);
            
            int npts = mSba->tracks.size();

            ROS_INFO("Bad projs (> 10 pix): %d, Cost without: %f",(int)mSba->countBad(10.0), sqrt(mSba->calcCost(10.0)/npts));
            ROS_INFO("Bad projs (> 5 pix): %d, Cost without: %f", (int)mSba->countBad(5.0), sqrt(mSba->calcCost(5.0)/npts));
            ROS_INFO("Bad projs (> 2 pix): %d, Cost without: %f", (int)mSba->countBad(2.0), sqrt(mSba->calcCost(2.0)/npts));
            ROS_INFO("Cameras (nodes): %d, Points: %d", (int)mSba->nodes.size(), (int)mSba->tracks.size());
                
            // Publish markers
            drawGraph(*mSba, mCameraMarkerPub, mPointMarkerPub);
            ros::spinOnce();

            return true;
        }


        //---------------------------------------------------------------------------------------------------------------------
        template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
        inline void BundleAdjusterRos<PointType_, DebugLevel_, OutInterface_>::recoverCameras(){
            for(unsigned i = 0; i < mSba->nodes.size(); i++){
                Eigen::Matrix4f newPose = Eigen::Matrix4f::Identity();
                Vector4d temptrans = mSba->nodes[i].trans;
                Quaterniond tempqrot = mSba->nodes[i].qrot;
                newPose.block<4,1>(0,3) = temptrans.cast<float>();
                newPose.block<3,3>(0,0) = tempqrot.matrix().cast<float>();

                auto df = this->mDataframes[this->mClustersIdxToId[i]]; 

                Eigen::Matrix4f offsetCluster = sf->pose().inverse()*newPose;
                
                df->updatePose(newPose);
            }
        }

    //---------------------------------------------------------------------------------------------------------------------
    template <typename PointType_, DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    inline void BundleAdjusterRos<PointType_, DebugLevel_, OutInterface_>::recoverPoints(){
       int nBadProj=0;
       for(unsigned i = 0; i < this->mWordIdxToId.size(); i++){
            int id = this->mWordIdxToId[i];

            this->mGlobalUsedWordsRef[id]->point.resize(3);

            this->mGlobalUsedWordsRef[id]->point[0] = (float) mSba->tracks[i].point[0];
            this->mGlobalUsedWordsRef[id]->point[1] = (float) mSba->tracks[i].point[1];
            this->mGlobalUsedWordsRef[id]->point[2] = (float) mSba->tracks[i].point[2];

            this->mGlobalUsedWordsRef[id]->optimized = true;
            
        }
        this->warning("BA_CVSBA","Num of bad points (behind camera) " + std::to_string(mSba->numBadPoints()));
        this->warning("BA_CVSBA","Deleted " + std::to_string(nBadProj) + " projections over " + std::to_string(mSba->countProjs()));
    }
#endif