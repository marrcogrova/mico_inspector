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
#include <sensor_msgs/NavSatFix.h>
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
#include <mico/base/map3d/Dataframe.h>
#include <mico/base/map3d/LoopClosureDetector.h>
#include <mico/base/utils/LogManager.h>
#include <mico/base/map3d/BundleAdjuster.h>
#include <mico/base/map3d/LoopClosureDetectorDorian.h>
#include <mico/base/map3d/DatabaseMarkI.h>

#include <mico/base/map3d/Odometry.h>
#include <mico/base/map3d/OdometryPhotogrammetry.h>
#include "EKFImu.h"


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

    private:
        void imageCallback(cv::Mat _image, float _altitude);

        bool ObtainPointCloud(float _altitude, std::vector<cv::KeyPoint> _keypoints, pcl::PointCloud<PointType_>::Ptr _OutputPointCloud);
		bool createVocabulary();

    private:

        bool savedFirstAltitude_ = false;
        bool imgIsRaw_;
        bool saveLogs_;
        bool publishPointCloud_;
        float altitude_;
        float firstAltitude_;
        float initSLAMAltitude_ = 5.0; // UAV altitude used to inicializate SLAM
        Eigen::Quaternionf lastOrientation_;
        Eigen::Vector3d ImuAcceleration_= Eigen::Vector3d::Identity();
        Eigen::Matrix4f firstPose_      = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f OdomPose_       = Eigen::Matrix4f::Identity();
        Eigen::Vector3f GPSposition_    = Eigen::Vector3f::Identity();

        cv::Mat intrinsics_,coefficients_;

        cv::Ptr<cv::ORB> ORBdetector_;
  
        ros::Subscriber imageSub_;
        ros::Subscriber infoSub_;
        ros::Subscriber imuSub_;
        ros::Subscriber GPSSub_;

        image_transport::Publisher featurePub_;
        ros::Publisher cloudPub_;
        ros::Publisher mapPub_;
        ros::Publisher posePub_;
        ros::Publisher markersVO_; // markers ClusterFrame
        ros::Publisher markersEKF_; // markers ClusterFrame
        
        visualization_msgs::Marker lineStrip_,EKFlineStrip_;

        int dfCounter_ = 0;

        std::ofstream logVO_,logEKF_; // Log in format TUM

        mico::Odometry<PointType_, mico::DebugLevels::Debug> *odometry_;
        // mico::DatabaseCF<PointType_, mico::DebugLevels::Debug> *database_;
        mico::DatabaseMarkI<PointType_, mico::DebugLevels::Debug> *database_;
        mico::BundleAdjuster<PointType_, mico::DebugLevels::Debug> *BA_;
        mico::LoopClosureDetector<> *loopDetector_ = nullptr;

        EKFImu ekf;
        std::chrono::time_point<std::chrono::system_clock> prevT_;
};

 #endif // MONO2RGBD_H_