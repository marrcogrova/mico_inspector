//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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

#ifndef MICO_FLOW_BLOCKS_STREAMERS_ROS_ROSSTREAMERS_H_
#define MICO_FLOW_BLOCKS_STREAMERS_ROS_ROSSTREAMERS_H_

#include <mico/flow/blocks/streamers/ros/BlockROSSuscriber.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifdef MICO_USE_ROS
	#include <geometry_msgs/PoseStamped.h>
	#include <geometry_msgs/Pose.h>
	#include <sensor_msgs/Image.h>
	#include <sensor_msgs/Imu.h>
	#include <sensor_msgs/PointCloud2.h>
	#include <pcl_conversions/pcl_conversions.h>
	#include <cv_bridge/cv_bridge.h>
#endif

namespace mico{
	#ifdef MICO_USE_ROS
    	// Declaration of conversion callbacks
    	Eigen::Matrix4f    PoseToMatrix4f(const geometry_msgs::Pose::ConstPtr &_msg);
    	Eigen::Matrix4f    PoseStampedToMatrix4f(const geometry_msgs::PoseStamped::ConstPtr &_msg);
		Eigen::Quaternionf ImuToQuaternionf(const sensor_msgs::Imu::ConstPtr &_msg);
		Eigen::Vector3f    ImuToAcceleration(const sensor_msgs::Imu::ConstPtr &_msg);
    	cv::Mat            RosImageToCvImage(const sensor_msgs::Image::ConstPtr &_msg);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ROSPointCloudToPCL(const sensor_msgs::PointCloud2::ConstPtr &_msg);

		std::function < std::any(const sensor_msgs::Imu::ConstPtr &) >           cbQuat        = ImuToQuaternionf;
		std::function < std::any(const sensor_msgs::Imu::ConstPtr &) >           cbAcc         = ImuToAcceleration;
		std::function < std::any(const sensor_msgs::Image::ConstPtr &) >         cbImage       = RosImageToCvImage;
		std::function < std::any(const sensor_msgs::PointCloud2::ConstPtr &) >   cbCloud       = ROSPointCloudToPCL;
		std::function < std::any(const geometry_msgs::Pose::ConstPtr &) >        cbPose        = PoseToMatrix4f;
		std::function < std::any(const geometry_msgs::PoseStamped::ConstPtr &) > cbPoseStamped = PoseStampedToMatrix4f;

    	// Declaration of blocks
		BlockPolicy<sensor_msgs::Imu > pol("Ros Imu Subscriber" , {"orientation" , "acceleration"} , {cbQuat , cbAcc} );
		typedef BlockROSSuscriber< &pol , sensor_msgs::Imu > BlockRosImu;

		BlockPolicy<sensor_msgs::PointCloud2 > polPCL("Ros Pointcloud Subscriber" , {"cloud"} , {cbCloud} );
		typedef BlockROSSuscriber< &polPCL , sensor_msgs::PointCloud2 > BlockRosCloud;

		BlockPolicy<geometry_msgs::Pose > polPose("Ros Pose Subscriber" , {"pose"} , {cbPose} );
		typedef BlockROSSuscriber< &polPose , geometry_msgs::Pose > BlockRosPose;

		BlockPolicy<geometry_msgs::PoseStamped > polPoseStamped("Ros PoseStamped Subscriber" , {"pose stamped"} , {cbPoseStamped} );
		typedef BlockROSSuscriber< &polPoseStamped , geometry_msgs::PoseStamped > BlockRosPoseStamped;

		BlockPolicy<sensor_msgs::Image > polImage("Ros Image Subscriber" , {"color"} , {cbImage} );
		typedef BlockROSSuscriber< &polImage , sensor_msgs::Image > BlockRosImage;			

	#endif
}

#endif