//---------------------------------------------------------------------------------------------------------------------
//  MONO2RGBD
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018  Ricardo Lopez Lopez (a.k.a. ricloplop) & Pablo Ramon Soria (a.k.a. Bardo91) & Marco Montes Grova (a.k.a marrcogrova)
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

#ifndef MONO2RGBD_H_
#define MONO2RGBD_H_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include "opencv2/opencv.hpp"
#include <opencv2/line_descriptor/descriptor.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mico/base/utils/LogManager.h>
#include <mico/base/map3d/DataFrame.h>
#include <mico/base/map3d/LoopClosureDetector.h>
#include <mico/base/utils/LogManager.h>
#include <mico/base/map3d/BundleAdjuster.h>
//#include <mico/base/map3d/LoopClosureDetectorDorian.h>
#include <mico/base/map3d/Database.h>

#include <mico/base/map3d/Odometry.h>
#include "OdometryPhotogrametry.h"
#include "EkfImuIcp.h"


#include "Eigen/Core"
#include "Eigen/Geometry"

#include <iostream>
#include <thread>
#include <boost/thread/thread.hpp>
#include <string>
#include <chrono>
#include <fstream>


class Mono2RGBD : public mico::LoggableInterface<mico::DebugLevels::Debug, mico::OutInterfaces::Cout> {
    public:
        typedef pcl::PointXYZRGBNormal PointType_;

        bool init(int _arc, char **_argv);
        
        bool step();

    protected:

        // Camera data callbacks 
        void imageCb(const sensor_msgs::Image::ConstPtr& _msg);
        void infoCb(const sensor_msgs::CameraInfo::ConstPtr& _msg);

        // UAV data callbacks 
        void imuCb(const sensor_msgs::Imu::ConstPtr& _msg);
        void poseCb(const geometry_msgs::PoseStamped::ConstPtr& _msg);

        bool ObtainPointCloud(std::vector<double> camera_center, double focalL, double cam_height , Eigen::Vector3d ea , std::vector<cv::KeyPoint> keypoints, pcl::PointCloud<PointType_>::Ptr OutputPointCloud);
        bool PublishUAV_Path(Eigen::Vector3f Position , Eigen::Quaternionf Orientation);
		bool createVocabulary();

    private:

        bool savefirstPosition_=true,savefirstOrient_=true,img_is_raw,_save_logs,_publish_pointCloud;
        Eigen::Vector3f lastPosition_;
        Eigen::Quaternionf lastOrientation_;
        Eigen::Vector3d ImuAcceleration_=Eigen::Vector3d::Identity();
        Eigen::Matrix4f firstPose_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f OdomPose_ = Eigen::Matrix4f::Identity();
        cv::Mat intrinsics_,coefficients_;
        nav_msgs::Path pathUAV_msg_;

        cv::Ptr<cv::ORB> ORBdetector_;
        
        tf::TransformBroadcaster br_,brOdom_;
  
        ros::Subscriber imageSub_;
        ros::Subscriber infoSub_;
        ros::Subscriber poseSub_;
        ros::Subscriber imuSub_;

        image_transport::Publisher featurePub_;
        ros::Publisher posePub_;
        ros::Publisher cloudPub_;
        ros::Publisher mapPub_;
        ros::Publisher markersCf; // markers ClusterFrame
        ros::Publisher markersEKF; // markers ClusterFrame
        ros::Publisher pathDataPub_;
        
        visualization_msgs::Marker lineStrip_,EKFlineStrip_;

        int dfCounter_ = 0;

        std::ofstream logGT_,logVO_,logEKF_; // Log in format TUM

    private:

        mico::Odometry<PointType_, mico::DebugLevels::Debug> *mOdometry;
        mico::Database<PointType_, mico::DebugLevels::Debug> *mDatabase;
        mico::BundleAdjuster<PointType_, mico::DebugLevels::Debug> *mBA;
        //mico::LoopClosureDetector<> *mLoopDetector = nullptr;

        EkfImuIcp ekf;
};

 #endif // MONO2RGBD_H_